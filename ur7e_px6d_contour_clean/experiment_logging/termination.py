"""Best-effort stop diagnostics, independent of motion and device lifetime.

observe/set_stop_reason do no I/O. Call flush only after existing stop actions.
The first terminal event stays authoritative; later failures retain their own
tracebacks without replacing the initial cause.
"""
from __future__ import annotations

from collections import Counter
from datetime import datetime, timezone
from enum import Enum
import json
import math
from pathlib import Path
import re
import sys
import tempfile
import time
import traceback

import numpy as np


class TerminationReason(str, Enum):
    SUCCESS = "SUCCESS"
    STOP_USER_REQUEST = "STOP_USER_REQUEST"
    STOP_FORCE_LIMIT = "STOP_FORCE_LIMIT"
    STOP_WORKSPACE_LIMIT = "STOP_WORKSPACE_LIMIT"
    STOP_UNEXPECTED_CONTACT = "STOP_UNEXPECTED_CONTACT"
    STOP_NO_CONTACT = "STOP_NO_CONTACT"
    STOP_RECOVERY_EXHAUSTED = "STOP_RECOVERY_EXHAUSTED"
    STOP_NO_FORWARD_PROGRESS = "STOP_NO_FORWARD_PROGRESS"
    STOP_REPEATED_CONTACT = "STOP_REPEATED_CONTACT"
    STOP_FIT_FAILURE = "STOP_FIT_FAILURE"
    STOP_ANCHOR_ERROR = "STOP_ANCHOR_ERROR"
    STOP_MOTION_ERROR = "STOP_MOTION_ERROR"
    STOP_SENSOR_ERROR = "STOP_SENSOR_ERROR"
    STOP_UNLOAD_NO_MOTION = "STOP_UNLOAD_NO_MOTION"
    STOP_UNLOAD_INEFFECTIVE = "STOP_UNLOAD_INEFFECTIVE"
    STOP_UNLOAD_BUDGET = "STOP_UNLOAD_BUDGET"
    STOP_FORCE_DIRECTION_UNVERIFIED = "STOP_FORCE_DIRECTION_UNVERIFIED"
    STOP_UNKNOWN_REASON = "STOP_UNKNOWN_REASON"
    STOP_DIRECTION_UNCONFIRMED = "STOP_DIRECTION_UNCONFIRMED"
    STOP_DIRECTION_REVERSAL = "STOP_DIRECTION_REVERSAL"
    STOP_DIRECTION_NO_PROGRESS = "STOP_DIRECTION_NO_PROGRESS"
    STOP_STALE_DATA = "STOP_STALE_DATA"
    STOP_INVALID_WRENCH = "STOP_INVALID_WRENCH"
    STOP_SEARCH_LIMIT = "STOP_SEARCH_LIMIT"
    STOP_TIME_LIMIT = "STOP_TIME_LIMIT"
    STOP_CONFIG_ERROR = "STOP_CONFIG_ERROR"
    STOP_POINT_LIMIT = "STOP_POINT_LIMIT"


def _chain(exception):
    seen = set()
    while exception is not None and id(exception) not in seen:
        seen.add(id(exception))
        yield exception
        exception = exception.__cause__ or (None if exception.__suppress_context__ else exception.__context__)


def _best_effort(operation, default=None):
    try:
        return operation()
    except Exception:
        return default


def _text(value):
    return _best_effort(lambda: str(value), f"<unprintable {type(value).__name__}>")


def _traceback_in_memory(exception):
    """All frames and chained exceptions, without linecache/source-file I/O."""
    lines = []
    chain = list(_chain(exception))
    for index, exc in enumerate(reversed(chain)):
        if index:
            outer = chain[len(chain) - index - 1]
            lines.append("\nThe above exception was the direct cause of the following exception:\n\n"
                         if outer.__cause__ is not None else "\nDuring handling of the above exception, another exception occurred:\n\n")
        if exc.__traceback__ is not None:
            lines.append("Traceback (most recent call last):\n")
            for frame, lineno in traceback.walk_tb(exc.__traceback__):
                lines.append(f'  File "{frame.f_code.co_filename}", line {lineno}, in {frame.f_code.co_name}\n')
        lines.append(f"{type(exc).__module__}.{type(exc).__name__}: {_text(exc)}\n")
    return "".join(lines)


def _has_nonfinite(value):
    if value is None:
        return False
    try:
        value = value.array() if hasattr(value, "array") else value
        return bool(np.any(~np.isfinite(np.asarray(value, dtype=float))))
    except (TypeError, ValueError):
        return False


def classify_stop_reason(detail="", exception=None, context=None):
    """Classify known reasons or typed evidence; free-form failures stay unknown."""
    if isinstance(detail, TerminationReason):
        return detail
    context = context or {}
    text = str(detail).strip()
    try:
        return TerminationReason(text)
    except ValueError:
        pass
    messages = [text, *(str(exc) for exc in _chain(exception))]
    # A later I/O/cleanup exception may carry the last, already-failed sample.
    # Use that sample to disambiguate data checks, not to explain an unrelated
    # new exception (for example a log writer failing after a NaN sensor stop).
    inspect_samples = exception is None or any(
        "nonfinite" in message.lower() or "invalid wrench" in message.lower()
        for message in messages)
    if inspect_samples:
        for field in ("actual_raw_wrench", "actual_processed_wrench", "raw_wrench", "processed_wrench", "raw", "processed"):
            if _has_nonfinite(context.get(field)):
                return TerminationReason.STOP_INVALID_WRENCH
        robot = context.get("robot")
        if any(_has_nonfinite(v) for v in (context.get("tcp_pose"), context.get("tcp_speed"),
                                           getattr(robot, "pose", None), getattr(robot, "tcp_speed", None))):
            return TerminationReason.STOP_MOTION_ERROR
    return_phases = ("SAFE_RETURN", "STARTUP_RETURN", "RETURN_TO_START", "VERTICAL_RETREAT", "MOVE_ABOVE_START", "DESCEND_TO_START")
    phase = str(context.get("phase", "")).upper()
    source = str(context.get("source", "")).lower()
    return_context = any(phase == p or phase.startswith(p + "_") for p in return_phases) or any(
        source == p or source.startswith(p + ".") for p in
        ("safe_return", "safety.safe_return", "app.safe_return", "app.startup_return", "return_to_start", "return_to_reset")
    )
    serialized_types = []
    exact = {
        "local contact trajectory loop closed": "SUCCESS",
        "optional loop closure detected": "SUCCESS",
        "Full contour loop completed successfully.": "SUCCESS",
        "two tracking probes and two returned recovery rays exercised": "SUCCESS",
        "operator stop": "STOP_USER_REQUEST", "operator normal stop": "STOP_USER_REQUEST",
        "Q normal stop": "STOP_USER_REQUEST", "ESC emergency stop": "STOP_USER_REQUEST",
        "Ctrl+C": "STOP_USER_REQUEST", "START not confirmed": "STOP_USER_REQUEST",
        "START_AIR not confirmed": "STOP_USER_REQUEST", "manual normal stop": "STOP_USER_REQUEST",
        "manual emergency stop": "STOP_USER_REQUEST", "finalized by operator": "STOP_USER_REQUEST",
        "operator stop in place": "STOP_USER_REQUEST",
        "unexpected contact during anchor transfer": "STOP_UNEXPECTED_CONTACT",
        "target acquisition failed after return to search anchor": "STOP_NO_CONTACT",
        "boundary lost; recovery disabled": "STOP_NO_CONTACT",
        "tracking unstable contact after two same-ray retries": "STOP_NO_CONTACT",
        "boundary recovery exhausted expanded local sectors": "STOP_RECOVERY_EXHAUSTED",
        "boundary point limit reached": "STOP_POINT_LIMIT",
        "maximum search distance reached": "STOP_SEARCH_LIMIT",
        "policy runtime limit reached": "STOP_TIME_LIMIT",
        "maximum experiment duration reached": "STOP_TIME_LIMIT",
        "simulation time limit exceeded": "STOP_TIME_LIMIT",
        "simulated TCP left container/workspace": "STOP_WORKSPACE_LIMIT",
        "return force limit exceeded": "STOP_FORCE_LIMIT",
        "return torque limit exceeded": "STOP_FORCE_LIMIT",
        "processed force safety threshold exceeded": "STOP_FORCE_LIMIT",
        "processed torque safety threshold exceeded": "STOP_FORCE_LIMIT",
        "absolute raw force safety threshold exceeded": "STOP_FORCE_LIMIT",
        "absolute raw torque safety threshold exceeded": "STOP_FORCE_LIMIT",
        "absolute raw force threshold exceeded during return": "STOP_FORCE_LIMIT",
        "absolute raw torque threshold exceeded during return": "STOP_FORCE_LIMIT",
        "air validation: real unfiltered force increment reached contact threshold": "STOP_FORCE_LIMIT",
        "air validation: real PX6D force rate exceeded": "STOP_FORCE_LIMIT",
        "air validation: actual TCP outside local XY envelope": "STOP_WORKSPACE_LIMIT",
        "air validation: next increment reaches local XY envelope": "STOP_WORKSPACE_LIMIT",
        "air validation: transfer target outside local XY envelope": "STOP_WORKSPACE_LIMIT",
        "air validation: overall timeout": "STOP_TIME_LIMIT",
        "anchor return failed": "STOP_ANCHOR_ERROR",
        "return to anchor timed out": "STOP_ANCHOR_ERROR",
        "cannot start another episode before anchor return": "STOP_ANCHOR_ERROR",
        "safe return requires normal stop": "STOP_MOTION_ERROR",
    }
    for message in messages:
        message = message.removeprefix("real PX6D: ")
        # Only a known return adapter may supply serialized exception types.
        # Untrusted/free-form strings cannot masquerade as a sensor/robot error.
        serialized = re.fullmatch(r"(ReturnAborted|RobotError|PX6DError|ProtocolError): (.+)", message)
        if return_context and serialized:
            serialized_types.append(serialized.group(1))
            message = serialized.group(2)
        if message in exact:
            return TerminationReason(exact[message])
        for prefix, reason in (
            ("NO_CONTACT", "STOP_NO_CONTACT"), ("NO_FORWARD_PROGRESS", "STOP_NO_FORWARD_PROGRESS"),
            ("NO_FORWARD_LOCAL_PROGRESS", "STOP_NO_FORWARD_PROGRESS"),
            ("REPEATED_CONTACT", "STOP_REPEATED_CONTACT"), ("FIT_FAILURE", "STOP_FIT_FAILURE"),
            ("CONFIRMATION_NO_CONTACT", "STOP_NO_CONTACT"), ("CONFIRMATION_NO_DISPLACEMENT", "STOP_REPEATED_CONTACT"),
            ("CONFIRMATION_UNSTABLE_TANGENT", "STOP_FIT_FAILURE"), ("CONFIRMATION_REVERSED", "STOP_FIT_FAILURE"),
        ):
            if message == prefix or message.startswith(prefix + ":"):
                return TerminationReason(reason)
        if re.match(r"^force rate exceeded: [-+0-9.]", message):
            return TerminationReason.STOP_FORCE_LIMIT
        if re.match(r"^headless step limit reached \(\d+\)$", message) or message.startswith("local initialization timeout: round="):
            return TerminationReason.STOP_TIME_LIMIT
        if message.startswith("local initialization retries exhausted: "):
            return TerminationReason.STOP_FIT_FAILURE
        if re.match(r"^TCP [xyz]=.+ is outside configured workspace$", message):
            return TerminationReason.STOP_WORKSPACE_LIMIT
        return_phase = r"(?:VERTICAL_RETREAT|MOVE_ABOVE_START|DESCEND_TO_START)"
        number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
        if re.fullmatch(return_phase + r" timed out", message):
            return TerminationReason.STOP_TIME_LIMIT
        if re.fullmatch(return_phase + r" settled (?:position|orientation) error " + number + r" (?:m|rad)", message):
            return TerminationReason.STOP_ANCHOR_ERROR
        if re.fullmatch(r"final (?:P0|RESET) (?:position|orientation) error " + number + r" (?:m|rad)", message):
            return TerminationReason.STOP_ANCHOR_ERROR
    phases = " ".join(str(context.get(name, "")) for name in ("phase", "operation", "source")).lower()
    for exc in _chain(exception):
        classes = [(cls.__name__, cls.__module__) for cls in type(exc).__mro__]
        if isinstance(exc, KeyboardInterrupt):
            return TerminationReason.STOP_USER_REQUEST
        if any(name in {"PX6DError", "ProtocolError"} and module == "sensor.px6d_reader" for name, module in classes):
            return TerminationReason.STOP_SENSOR_ERROR
        if any(name in {"SerialException", "SerialTimeoutException"} and module.startswith("serial") for name, module in classes):
            return TerminationReason.STOP_SENSOR_ERROR
        if any(name == "CalibrationError" and module == "calibration.scan_calibration" for name, module in classes):
            return TerminationReason.STOP_CONFIG_ERROR
        if re.search(r"config|calibration|preflight|validate_execution", phases) and isinstance(exc, (ValueError, KeyError, OSError)):
            return TerminationReason.STOP_CONFIG_ERROR
        if re.search(r"sensor[_ .]?(read|connect)|read_wrench|px6d[_ .]?(read|connect)", phases) and isinstance(exc, (ValueError, OSError)):
            return TerminationReason.STOP_SENSOR_ERROR
        if any(name == "RobotError" and module == "robot.rtde_controller" for name, module in classes):
            return TerminationReason.STOP_MOTION_ERROR
    if any(name in {"PX6DError", "ProtocolError"} for name in serialized_types):
        return TerminationReason.STOP_SENSOR_ERROR
    if "RobotError" in serialized_types:
        return TerminationReason.STOP_MOTION_ERROR
    return TerminationReason.STOP_UNKNOWN_REASON


def _safe(value, depth=0):
    """Strict JSON values; nonfinite measurements remain explicit diagnostics."""
    if depth > 12:
        return "<maximum diagnostic depth>"
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else ("NaN" if math.isnan(value) else ("+Infinity" if value > 0 else "-Infinity"))
    if isinstance(value, np.generic):
        return _safe(value.item(), depth + 1)
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_safe(item, depth + 1) for item in value]
    if isinstance(value, dict):
        return {str(key): _safe(item, depth + 1) for key, item in value.items()}
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, "array"):
        return _safe(value.array(), depth + 1)
    try:
        return repr(value)
    except Exception:
        return "<unprintable diagnostic value>"


class TerminationRecorder:
    def __init__(self, output_root=None, print_fn=print, *, mode=None, strategy=None, config_source=None):
        self.output_root = output_root
        self.identity = (mode, strategy, config_source)
        self.print_fn = print_fn
        self.run_dir = None
        self._observed = {}
        self._record = None
        self.events = []
        self.secondary_errors = []
        self._emitted_version = None

    @property
    def record(self):
        return self._record

    def observe(self, *, policy=None, robot=None, raw=None, processed=None, command=None,
                state=None, phase=None, timestamp=None, **extra):
        # Keep references only. Snapshots are taken on events, never each tick.
        self._observed.update({key: value for key, value in dict(
            policy=policy, robot=robot, raw=raw, processed=processed, command=command,
            state=state, phase=phase, timestamp=timestamp, **extra).items() if value is not None})

    def _snapshot(self, context):
        refs = {**self._observed, **context}
        policy, robot, command = (refs.get(key) for key in ("policy", "robot", "command"))
        active_episode = getattr(policy, "active_episode", None)
        episode = active_episode
        episodes = getattr(policy, "probe_episodes", [])
        if episode is None and episodes:
            episode = episodes[-1]
        recovery = getattr(policy, "recovery", None)
        queue = getattr(policy, "_motion_queue", [])
        state = getattr(getattr(policy, "state", None), "value", getattr(policy, "state", None))
        raw, processed = refs.get("raw"), refs.get("processed")
        raw_values = raw.array() if hasattr(raw, "array") else raw
        processed_values = processed.array() if hasattr(processed, "array") else processed
        if refs.get("force_input_mode") == "synthetic_policy_fixture":
            raw_values = refs.get("actual_raw_wrench", raw_values)
            processed_values = refs.get("actual_processed_wrench", processed_values)
        force_values = raw_values if processed_values is None else processed_values
        force_source = "processed_wrench" if processed_values is not None else ("raw_wrench" if raw_values is not None else "unavailable")
        force_frame = (refs.get("processed_force_frame", "configured_output_frame") if processed_values is not None
                       else ("sensor_frame" if raw_values is not None else "unavailable"))
        anchor = getattr(active_episode, "anchor_pose", None)
        if getattr(active_episode, "phase", None) == "RETURN":
            return_target = getattr(active_episode, "return_target_pose", None)
            if return_target is not None:
                anchor = return_target
        if anchor is None and queue:
            anchor = queue[0][0]
        # Recovery objects remain available as history after tracking resumes;
        # their frozen anchor describes the current action only in these states.
        if anchor is None and state in {"BOUNDARY_RECOVERY", "BOUNDARY_CONFIRMATION"}:
            anchor = getattr(recovery, "anchor", None)
        if anchor is None:
            anchor = getattr(episode, "return_target_pose", None)
        if anchor is None:
            anchor = getattr(episode, "anchor_pose", None)
        recovery_records = getattr(policy, "recovery_records", [])
        recovery_id = (len(recovery_records) - 1 if recovery is not None and recovery_records
                       else getattr(episode, "recovery_id", None))
        recovery_probes = [{name: getattr(item, name, None) for name in
                           ("probe_id", "purpose", "phase", "outcome", "return_completed", "rejection_reason")}
                           for item in episodes if recovery is not None and recovery_id is not None
                           and getattr(item, "recovery_id", None) == recovery_id
                           and getattr(item, "purpose", None) in {"RECOVERY", "CONFIRMATION"}]
        last_contact = None
        for item in reversed(episodes):
            if getattr(item, "contact_pose", None) is not None:
                last_contact = item.contact_pose
                break
        if last_contact is None and getattr(policy, "boundary_points", []):
            last_contact = policy.boundary_points[-1].pose
        diagnostic_state = state if state is not None else refs.get("state", getattr(command, "state", None))
        if getattr(command, "state", None) == "RETURN_TO_START":
            diagnostic_state = "RETURN_TO_START"
        elif queue:
            diagnostic_state = "ANCHOR_TRANSFER"
        command_data = None if command is None else {name: getattr(command, name, None) for name in
            ("state", "policy_sub_state", "move", "direction_xy", "speed", "target_pose", "contact_flag", "reason", "probe_id")}
        excluded = {"policy", "robot", "raw", "processed", "command", "timestamp"}
        snapshot = {key: value for key, value in refs.items() if key not in excluded}
        snapshot.update({
            "state": diagnostic_state, "policy_state": state,
            "policy_sub_state": getattr(policy, "sub_state", None),
            "phase": refs.get("phase", getattr(episode, "phase", None)),
            "tcp_pose": getattr(robot, "pose", refs.get("tcp_pose")),
            "tcp_speed": getattr(robot, "tcp_speed", refs.get("tcp_speed")),
            "robot_sample_timestamp": getattr(robot, "timestamp", None),
            "anchor_pose": anchor, "last_contact": last_contact,
            "probe_id": getattr(episode, "probe_id", None), "probe_phase": getattr(episode, "phase", None),
            "probe_purpose": getattr(episode, "purpose", None),
            "probe_anchor_pose": getattr(episode, "anchor_pose", None),
            "probe_return_target_pose": getattr(episode, "return_target_pose", None),
            "probe_outcome": getattr(episode, "outcome", None),
            "probe_return_completed": getattr(episode, "return_completed", None),
            "max_probe_distance": getattr(episode, "max_probe_distance", None),
            "probe_direction": getattr(episode, "probe_direction", None),
            "candidate_status": getattr(command, "candidate_status", None),
            "rejection_reason": getattr(episode, "rejection_reason", getattr(command, "rejection_reason", None)),
            "recovery_id": recovery_id,
            "recovery_attempted_count": len(getattr(recovery, "attempted_directions", [])),
            "recovery_candidate_count": len(getattr(recovery, "candidates", [])),
            "recovery_probe_summary": recovery_probes,
            "recovery_outcome_counts": dict(Counter(item["outcome"] for item in recovery_probes)),
            "recovery_rejection_counts": dict(Counter(item["rejection_reason"] for item in recovery_probes if item["rejection_reason"])),
            "raw_wrench": raw_values, "processed_wrench": processed_values,
            "actual_force": None if force_values is None else np.asarray(force_values)[:3],
            "force_source": force_source, "force_frame": force_frame,
            "policy_processed_wrench": None if processed is None else processed,
            "command": command_data,
        })
        return snapshot, refs

    def set_stop_reason(self, reason=None, detail="", *, exception=None, source="", terminal=True, replace=False, **context):
        try:
            snapshot, refs = self._snapshot(context)
            source = source or refs.get("source", "")
            snapshot["source"] = source
            if reason is None:
                classified = classify_stop_reason(detail, exception, {**refs, **snapshot, "source": source or refs.get("source", "")})
            else:
                try:
                    classified = TerminationReason(reason)
                except ValueError:
                    classified = TerminationReason.STOP_UNKNOWN_REASON
                    if not detail:
                        detail = str(reason)
            event = _safe({**snapshot,
                "reason": classified.value, "detail": str(detail), "source": source,
                "terminal": bool(terminal), "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "monotonic_sec": refs.get("timestamp", time.monotonic()),
                "context": snapshot,
                "exception_type": None if exception is None else type(exception).__name__,
                "exception_chain": [{"type": type(exc).__module__ + "." + type(exc).__name__, "message": str(exc)} for exc in _chain(exception)],
                "traceback": "" if exception is None else _traceback_in_memory(exception),
            })
            self.events.append(event)
            if terminal and (self._record is None or replace):
                self._record = event
            elif exception is not None:
                self.secondary_errors.append(event)
            return event
        except Exception as diagnostic_error:
            # A malformed diagnostic object cannot replace the control failure.
            refs = {**self._observed, **context}
            policy, command = refs.get("policy"), refs.get("command")
            state = _best_effort(lambda: policy.state, refs.get("state"))
            state = _best_effort(lambda: state.value, state)
            event = _safe({"reason": TerminationReason.STOP_UNKNOWN_REASON.value, "detail": _text(detail),
                     "source": source, "terminal": bool(terminal), "diagnostic_error": _text(diagnostic_error),
                     "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                     "monotonic_sec": refs.get("timestamp", time.monotonic()), "state": state, "policy_state": state,
                     "command": None if command is None else {name: _best_effort(lambda name=name: getattr(command, name))
                         for name in ("state", "move", "direction_xy", "speed", "target_pose")},
                     "exception_type": None if exception is None else type(exception).__name__,
                     "traceback": "" if exception is None else _traceback_in_memory(exception),
                     "exception_chain": [{"type": type(exc).__module__ + "." + type(exc).__name__, "message": _text(exc)} for exc in _chain(exception)]})
            self.events.append(event)
            if terminal and self._record is None:
                self._record = event
            elif exception is not None:
                self.secondary_errors.append(event)
            return event

    def bind(self, run_dir):
        self.run_dir = Path(run_dir)
        return self

    def summary_fields(self):
        record = self.record
        return {"termination_reason": None if record is None else record["reason"],
                "termination_detail": None if record is None else record["detail"],
                "termination": record, "termination_context": None if record is None else record.get("context", record)}

    def _report_text(self, payload):
        lines = ["SCAN TERMINATED", f"Reason: {payload['reason']}",
                 f"State: {payload.get('state')} (policy={payload.get('policy_state')}, phase={payload.get('phase')})",
                 f"Detail: {payload.get('detail', '')}", f"TCP: {payload.get('tcp_pose')}",
                 f"Force: {payload.get('actual_force')} (source={payload.get('force_source')}, frame={payload.get('force_frame')})",
                 f"Last contact: {payload.get('last_contact')}",
                 f"Current anchor: {payload.get('anchor_pose')}",
                 f"Timestamp: {payload.get('timestamp_utc')} (monotonic={payload.get('monotonic_sec')})",
                 f"Source: {payload.get('source', '')}", f"Raw sensor: {payload.get('raw_wrench')}",
                 f"Diagnostics: {None if self.run_dir is None else self.run_dir / 'termination.json'}"]
        if payload.get("traceback"):
            lines.append(payload["traceback"])
        for error in self.secondary_errors:
            lines.append(f"SECONDARY {error.get('reason', 'STOP_UNKNOWN_REASON')}: {error.get('detail', '')} (source={error.get('source', '')})")
            if error.get("traceback"):
                lines.append(error["traceback"])
        return "\n".join(lines) + "\n"

    def _log_error(self, exc):
        event = self.set_stop_reason(TerminationReason.STOP_UNKNOWN_REASON,
                                     detail=f"termination logging failed: {type(exc).__name__}: {exc}",
                                     exception=exc, source="termination.flush", terminal=False, operation="logging")
        try:
            sys.stderr.write(event["detail"] + "\n")
        except Exception:
            pass

    def flush(self, emit=False):
        """Persist diagnostics; all persistence/console failures stay secondary."""
        if self.record is None:
            self.set_stop_reason(TerminationReason.STOP_UNKNOWN_REASON, "no terminal reason was recorded", source="termination.flush")
        for attempt in range(2):
            try:
                if self.run_dir is None:
                    mode, strategy, source = self.identity
                    if mode is None or strategy is None:
                        if emit:
                            self.print_fn(self._report_text(self.record))
                        return None
                    from experiment_logging.paths import create_run
                    self.run_dir = create_run(mode, strategy, source, data_root=self.output_root)
                self.run_dir.mkdir(parents=True, exist_ok=True)
                payload = {**self.record, "events": self.events, "secondary_errors": self.secondary_errors}
                encoded = json.dumps(_safe(payload), ensure_ascii=False, allow_nan=False, indent=2) + "\n"
                temporary = self.run_dir / "termination.json.tmp"
                temporary.write_text(encoded, encoding="utf-8")
                temporary.replace(self.run_dir / "termination.json")
                report = self._report_text(payload)
                (self.run_dir / "termination.txt").write_text(report, encoding="utf-8")
                summary_path = self.run_dir / "summary.json"
                if summary_path.is_file():
                    summary = json.loads(summary_path.read_text(encoding="utf-8"))
                    summary.update(self.summary_fields())
                    summary["termination_events"] = self.events
                    summary["termination_secondary_errors"] = self.secondary_errors
                    summary_temp = self.run_dir / "summary.json.tmp"
                    summary_temp.write_text(json.dumps(_safe(summary), ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")
                    summary_temp.replace(summary_path)
                break
            except Exception as exc:
                self._log_error(exc)
                break  # Disk failure is reported; never scatter fallback logs outside data/.
        if emit:
            version = (len(self.events), len(self.secondary_errors))
            if version != self._emitted_version:
                try:
                    self.print_fn(self._report_text(self.record))
                    self._emitted_version = version
                except Exception as exc:
                    self._log_error(exc)
                    self.flush(emit=False)
        return self.run_dir


def set_stop_reason(recorder, *args, **kwargs):
    """Functional adapter for callers that keep a recorder in their context."""
    return recorder.set_stop_reason(*args, **kwargs)
