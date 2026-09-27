"""Offline workspace projection of an existing real or simulated run."""
from __future__ import annotations

import argparse

from workspace.workspace_logger import export_existing_run
from workspace.workspace_transform import DEFAULT_CALIBRATION_PATH


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, help="directory containing samples.csv or simulation_log.csv")
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION_PATH))
    parser.add_argument("--output-dir", help="defaults to run directory; original scan logs are never changed")
    parser.add_argument("--overwrite", action="store_true", help="replace only existing workspace outputs")
    args = parser.parse_args(argv)
    try:
        path = export_existing_run(args.run_dir, args.calibration, args.output_dir, overwrite=args.overwrite)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f"Workspace export failed: {type(exc).__name__}: {exc}\n")
    print(f"Workspace CSV: {path}")
    print(f"Workspace image: {path.parent / 'workspace_contour.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
