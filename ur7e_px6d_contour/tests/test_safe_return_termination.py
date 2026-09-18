"""Safe-return diagnostics retain failing samples without changing motion order."""
from types import SimpleNamespace

import numpy as np
import pytest

from core.models import Wrench
from experiment_logging.termination import TerminationRecorder
from safety.safe_return import SafeReturnExecutor
from test_safe_return import FakeController, FakeReader, return_config


ZERO = Wrench(0, 0, 0, 0, 0, 0)
P0 = np.array([.1, .2, .2, 0, 0, 0])
START = np.array([.4, -.2, .2, 0, 0, 0])


def context(tmp_path, *, logger=False):
    def unexpected_output(*_):
        raise AssertionError("safe-return diagnostic hooks must not emit or flush")

    term = TerminationRecorder(output_root=tmp_path, print_fn=unexpected_output)
    term.flush = unexpected_output
    policy = SimpleNamespace(termination=term, state="RETURN_TO_START",
                             current_target_direction=None, current_tangent=None)
    log = SimpleNamespace(termination=term, run_dir=tmp_path, log_sample=lambda *args: None) if logger else None
    return term, policy, log


class Samples:
    def __init__(self, samples):
        self.samples = iter(samples)

    def read_wrench(self):
        return next(self.samples)


def test_legacy_abort_helper_also_uses_shared_reason(tmp_path):
    term, policy, _ = context(tmp_path)
    executor = SafeReturnExecutor(return_config(), P0, FakeController(START), FakeReader(), None,
                                  policy=policy)
    result = executor._aborted("return force limit exceeded", START)
    assert result.status == "aborted"
    assert term.record["reason"] == "STOP_FORCE_LIMIT"
    assert term.record["source"] == "safe_return._aborted"


@pytest.mark.parametrize("binding", ["policy", "logger"])
def test_force_abort_captures_failing_frame_before_stop_even_without_logger(tmp_path, binding):
    term, policy, logger = context(tmp_path, logger=binding == "logger")
    controller = FakeController(START)
    original_stop = controller.safe_stop_motion
    seen_at_stop = []

    def stop(force=False):
        seen_at_stop.append(term.record)
        original_stop(force)

    controller.safe_stop_motion = stop
    result = SafeReturnExecutor(return_config(), P0, controller,
                                Samples([ZERO, Wrench(9, 0, 0, 0, 0, 0)]), None,
                                logger=logger, policy=policy if binding == "policy" else None).execute()
    assert result.status == "aborted" and result.abort_reason == "ReturnAborted: return force limit exceeded"
    record = term.record
    assert record["reason"] == "STOP_FORCE_LIMIT"
    assert record["raw_wrench"] == [9, 0, 0, 0, 0, 0]
    assert record["processed_wrench"] == [9, 0, 0, 0, 0, 0]
    assert record["actual_force"] == [9, 0, 0]
    assert record["state"] == "RETURN_TO_START"
    assert record["phase"] == "VERTICAL_RETREAT_FORCE_CHECK"
    assert record["command"]["move"] is True
    assert record["command"]["speed"] == return_config()["safe_return"]["return_vertical_speed"]
    assert "ReturnAborted" in record["traceback"] and "_check_force" in record["traceback"]
    assert seen_at_stop[0] is None and seen_at_stop[1] is record
    assert len(controller.moves) == 1 and not controller.return_mode


def test_unknown_return_abort_retains_original_exception_chain(tmp_path):
    term, policy, _ = context(tmp_path)

    class BrokenReader:
        def read_wrench(self):
            try:
                raise LookupError("opaque original context")
            except LookupError as cause:
                raise RuntimeError("opaque return failure") from cause

    controller = FakeController(START)
    result = SafeReturnExecutor(return_config(), P0, controller, BrokenReader(), None,
                                policy=policy).execute()
    assert result.status == "aborted" and result.abort_reason == "RuntimeError: opaque return failure"
    record = term.record
    assert record["reason"] == "STOP_UNKNOWN_REASON"
    assert "RuntimeError: opaque return failure" in record["traceback"]
    assert "LookupError: opaque original context" in record["traceback"]
    assert len(record["exception_chain"]) == 2
    assert not controller.moves and not controller.return_mode


def test_return_failure_preserves_first_user_stop_and_adds_current_force_event(tmp_path):
    term, policy, _ = context(tmp_path)
    term.set_stop_reason("STOP_USER_REQUEST", "Q normal stop")
    first = term.record
    result = SafeReturnExecutor(return_config(), P0, FakeController(START),
                                Samples([ZERO, Wrench(9, 0, 0, 0, 0, 0)]), None,
                                policy=policy).execute()
    assert result.status == "aborted"
    assert term.record is first and first["reason"] == "STOP_USER_REQUEST"
    event = term.events[-1]
    assert event["source"] == "safe_return.execute" and event["terminal"] is False
    assert event["reason"] == "STOP_FORCE_LIMIT"
    assert event["raw_wrench"] == [9, 0, 0, 0, 0, 0]
    assert "_check_force" in event["traceback"]


def test_processing_exception_retains_just_read_raw_wrench(tmp_path):
    term, policy, _ = context(tmp_path)

    class BrokenPreprocessor:
        def process(self, raw):
            raise RuntimeError("unclassified return preprocessing error")

    result = SafeReturnExecutor(return_config(), P0, FakeController(START),
                                FakeReader(Wrench(2, 3, 4, 0, 0, 0)), BrokenPreprocessor(),
                                policy=policy).execute()
    assert result.status == "aborted"
    assert term.record["raw_wrench"] == [2, 3, 4, 0, 0, 0]
    assert term.record["phase"] == "VERTICAL_RETREAT_PRECHECK_PROCESSING"
    assert "process" in term.record["traceback"]
    assert "unclassified return preprocessing error" in term.record["traceback"]


def test_rejected_move_retains_attempted_target_and_speed(tmp_path):
    term, policy, _ = context(tmp_path)

    class BrokenMotion(FakeController):
        def move_linear_async(self, target, speed, acceleration):
            raise RuntimeError("opaque move rejection")

    result = SafeReturnExecutor(return_config(), P0, BrokenMotion(START), FakeReader(), None,
                                policy=policy).execute()
    assert result.status == "aborted"
    command = term.record["command"]
    assert command["move"] is True
    assert command["target_pose"][:3] == pytest.approx([.4, -.2, .23])
    assert command["speed"] == pytest.approx(.003)
    assert "move_linear_async" in term.record["traceback"]


def test_existing_api_without_recorder_preserves_all_return_segments():
    controller = FakeController(START)
    result = SafeReturnExecutor(return_config(), P0, controller, FakeReader(), None).execute()
    assert result.status == "complete"
    assert len(controller.moves) == 3
    assert controller.moves[0][0][:3] == pytest.approx([.4, -.2, .23])
    assert controller.moves[1][0][:3] == pytest.approx([.1, .2, .23])
    assert np.array_equal(controller.moves[2][0], P0)
    assert not controller.return_mode


def test_end_return_mode_error_is_still_propagated_to_caller(tmp_path):
    term, policy, _ = context(tmp_path)

    class BrokenEnd(FakeController):
        def end_return_mode(self):
            raise RuntimeError("end return mode failed")

    with pytest.raises(RuntimeError, match="end return mode failed"):
        SafeReturnExecutor(return_config(), P0, BrokenEnd(START), FakeReader(), None,
                           policy=policy).execute()
    # The successful return itself neither invents a scan cause nor suppresses
    # the original finally exception; the application records that exception.
    assert term.record is None
