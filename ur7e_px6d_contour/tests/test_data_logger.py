import csv

import numpy as np

from experiment_logging.data_logger import ExperimentLogger
from core.models import PolicyCommand, RobotState, Wrench


def test_sample_log_contains_policy_recovery_diagnostics(tmp_path):
    logger = ExperimentLogger(tmp_path, {"test": True})
    wrench = Wrench(1, 2, 3, 0.1, 0.2, 0.3)
    robot = RobotState(1.0, np.zeros(6), np.zeros(6))
    command = PolicyCommand(
        "BOUNDARY_RECOVERY",
        False,
        np.zeros(2),
        0.0,
        np.zeros(6),
        True,
        True,
        "candidate rejected",
        "RETURN_TO_ANCHOR",
        2,
        4,
        55.0,
        "REJECTED",
        "PREVIOUSLY_VISITED_EDGE",
    )
    logger.log_sample(1.0, wrench, wrench, robot, command, [1, 0], [0, 1])
    run_dir = logger.run_dir
    logger.close()

    with (run_dir / "samples.csv").open(newline="", encoding="utf-8") as handle:
        row = next(csv.DictReader(handle))
    assert row["policy_sub_state"] == "RETURN_TO_ANCHOR"
    assert row["interaction_direction_x"] != ""
    assert row["recovery_id"] == "2"
    assert row["ray_index"] == "4"
    assert row["candidate_status"] == "REJECTED"
    assert row["rejection_reason"] == "PREVIOUSLY_VISITED_EDGE"
    assert (run_dir / "boundary_recovery_rays.csv").is_file()
