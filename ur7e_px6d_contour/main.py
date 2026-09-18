#!/usr/bin/env python3
"""Compatibility entry point; orchestration lives in :mod:`app.main`."""

from app.main import capture_bias, cli_main, format_dry_run, parse_args, refresh_emergency_pose, run


if __name__ == "__main__":
    raise SystemExit(cli_main())
