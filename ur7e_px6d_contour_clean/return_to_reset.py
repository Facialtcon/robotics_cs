"""Compatibility entry; daily operation uses run_project.py."""
from app.return_runtime import main

if __name__ == "__main__":
    raise SystemExit(main())
