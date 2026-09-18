#!/usr/bin/env python3
"""Preview or explicitly arm the real UR7e/PX6D air validation."""
import sys
from app.real_validation import main, PROJECT_ROOT
from experiment_logging.termination import TerminationRecorder


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        recorder = getattr(exc, "termination_recorder", None) or TerminationRecorder(output_root=PROJECT_ROOT / "data")
        recorder.set_stop_reason(detail=f"{type(exc).__name__}: {exc}", exception=exc,
                                 source="run_real_validation")
        recorder.flush(emit=True)
        print(f"Air validation refused: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
