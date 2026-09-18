"""New, historical and unknown logged states must not lose the result figure."""
from pathlib import Path

import numpy as np
import yaml

from core.models import PolicyCommand, RobotState, Wrench
from experiment_logging.data_logger import ExperimentLogger
from policy.rule_policy import State
from tools.visualize_run import create_visualization, create_strategy_debug


def test_result_plot_handles_all_policy_states_and_empty_boundary(tmp_path):
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / "config.yaml").read_text())
    logger = ExperimentLogger(tmp_path, config)
    wrench = Wrench(0, 0, 0, 0, 0, 0)
    states = [s.value for s in State] + ["SEARCH", "CORNER_SEARCH", "FUTURE_STATE"]
    for index, state in enumerate(states):
        pose = np.array([index * .001, 0, .1, 0, 0, 0])
        command = PolicyCommand(state, False, np.zeros(2), 0, pose, False, False, "test")
        logger.log_sample(float(index), wrench, wrench,
                          RobotState(float(index), pose, np.zeros(6)), command, None, None)
    logger.close()
    path = create_visualization(logger.run_dir)
    assert path.is_file() and path.stat().st_size > 1000
    strategy, _ = create_strategy_debug(logger.run_dir)
    assert strategy.is_file()
