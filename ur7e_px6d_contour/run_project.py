#!/usr/bin/env python3
"""中文分阶段入口：只读/模拟与明确确认的真机运动分区。"""

from __future__ import annotations

import argparse
import importlib.util
import os
import csv
import json
import math
import shlex
from datetime import datetime
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def clean_child_environment() -> dict[str, str]:
    """Keep this non-ROS project isolated from a sourced ROS shell."""
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    # ROS installs pytest entry-point plugins globally.  The project tests do
    # not use third-party pytest plugins, so disable automatic discovery.
    environment["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    return environment

STEPS = {
    "1": ("检查代码（不连接任何设备）", [sys.executable, "-m", "pytest", "-q"]),
    "2": (
        "原离散策略：正方形整圈验证并保存一张轨迹图（不连接任何设备）",
        [sys.executable, "tools/run_mock_visualized.py"],
    ),
    "3": ("只读取 PX6D（机器人不会动）", [sys.executable, "tools/check_px6d.py", "--samples", "500"]),
    "4": ("只读取 UR7e 状态（机器人不会动）", [sys.executable, "tools/check_rtde.py"]),
    "5": ("原离散策略：真实 PX6D 驱动规则 dry-run（不连接机器人）", [sys.executable, "main.py", "--sensor", "real"]),
    "6": ("原离散策略：可选方/圆/三角并可拖动的实时动画（不连接任何设备）", [sys.executable, "run_simulation.py"]),
    "7": (
        "标定扫描起点 P0 和初始扫描方向（机器人不移动）",
        [sys.executable, "run_calibration.py"],
    ),
    "8": (
        "检查扫描标定结果（不连接任何设备）",
        [sys.executable, "tools/check_scan_calibration.py"],
    ),
    "9": (
        "读取 active TCP 并写入 config.yaml（备份原配置，不发送运动命令）",
        [sys.executable, "tools/read_active_tcp.py", "--write-config"],
    ),
    "10": (
        "将任意静止位置保存为 TCP 绑定复位点（不发送运动命令）",
        [sys.executable, "save_reset_pose.py"],
    ),
    "11": (
        "从当前位置返回复位目标；未另存时用扫描 P0（真机运动）",
        [sys.executable, "return_to_reset.py"],
    ),
    "12": (
        "原离散策略：UR7e + PX6D 轮廓扫描（真机运动）",
        [sys.executable, "main.py", "--sensor", "real", "--execute"],
    ),
}

STEPS.update({
    "13": ("箱体四角标定（只读取机器人；保存需子工具确认）", [sys.executable, "calibrate_workspace.py"]),
    "14": ("箱体四角离线检查（只读）", [sys.executable, "tools/check_workspace_calibration.py"]),
    "15": ("连续策略：交互预演（离线，手动停止）", [sys.executable, "run_continuous_tracking.py", "--preview"]),
    "16": ("连续策略：有限时长无窗口模拟（离线）", [sys.executable, "run_continuous_tracking.py", "--dry-run"]),
    "17": ("连续策略：已有标定与配置离线检查", [sys.executable, "run_continuous_tracking.py", "--check-calibration"]),
    "18": ("连续策略：UR7e + PX6D 扫描（真机运动）", [sys.executable, "run_continuous_tracking.py", "--execute"]),
    "19": ("选择已有运行目录，查看日志并静态回放", []),
    "20": ("选择已有运行目录，箱体坐标可视化", [sys.executable, "visualize_workspace.py"]),
})

MENU_GROUPS = (
    ("共用准备（只读硬件项仍会连接设备）", ("1", "3", "4", "7", "8", "9", "10", "13", "14", "17")),
    ("离线模拟 / 原离散 dry-run（5 连接真实传感器）", ("2", "5", "6", "15", "16")),
    ("真机运动（必须现场监护并确认）", ("11", "12", "18")),
    ("结果查看（离线）", ("19", "20")),
)

MOTION_CONFIRMATIONS = {
    "11": "OPEN_REAL_RESET",
    "12": "OPEN_REAL_SCAN",
    "18": "OPEN_CONTINUOUS_SCAN",
}


def print_menu() -> None:
    print("\nUR7e + PX6D 统一运行菜单")
    for title, steps in MENU_GROUPS:
        print(f"\n【{title}】")
        for step in steps:
            print(f"{step:>2}. {STEPS[step][0]}")
    print("\n0. 退出")
    print("连续 18：Q/Esc/Ctrl+C 原地停止，不自动返回。")
    print("原离散 12：Q 按原配置正常停止/返回；Esc/Ctrl+C 不自动返回。")
    print("11/12/18 须检查完整路径；三段返回没有碰撞规划。")


def check_dependency(step: str) -> str | None:
    required = {
        "1": ("pytest", "pytest"),
        "2": ("matplotlib", "matplotlib"),
        "3": ("serial", "pyserial"),
        "4": ("rtde_receive", "ur-rtde"),
        "5": ("serial", "pyserial"),
        "6": ("matplotlib", "matplotlib"),
        "7": ("rtde_receive", "ur-rtde"),
        "8": ("matplotlib", "matplotlib"),
        "9": ("rtde_control", "ur-rtde"),
        "10": ("rtde_control", "ur-rtde"),
        "11": ("rtde_control", "ur-rtde"),
        "12": ("rtde_control", "ur-rtde"),
        "13": ("rtde_receive", "ur-rtde"),
        "14": ("yaml", "PyYAML"),
        "15": ("matplotlib", "matplotlib"),
        "16": ("numpy", "numpy"),
        "17": ("yaml", "PyYAML"),
        "18": ("rtde_control", "ur-rtde"),
        "19": ("yaml", "PyYAML"),
        "20": ("matplotlib", "matplotlib"),
    }
    if step not in required:
        return None
    module, package = required[step]
    if importlib.util.find_spec(module) is None:
        return f"缺少 {package}。先运行：{sys.executable} -m pip install -r requirements.txt"
    return None


class Cancelled(Exception):
    pass


def ask(prompt: str) -> str:
    value = input(prompt + "（输入 :q 取消）：").strip()
    if value == ":q":
        raise Cancelled
    return value


def choose_path(prompt: str, default: Path | None = None, *, directory=False) -> Path:
    while True:
        value = ask(prompt + (f" [回车默认 {default}]" if default else ""))
        if not value and default is None:
            print("路径不能为空。")
            continue
        path = Path(value).expanduser() if value else default
        if not path.is_absolute():
            path = ROOT / path
        path = path.resolve()
        if (path.is_dir() if directory else path.is_file()):
            return path
        print(f"路径不存在或类型不符：{path}")


def read_mapping(path: Path) -> dict:
    if not path.exists():
        return {}
    if path.suffix == ".json":
        data = json.loads(path.read_text(encoding="utf-8"))
    else:
        import yaml
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
        except yaml.YAMLError as exc:
            raise ValueError(f"元数据 YAML 无效：{path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"元数据不是 mapping：{path}")
    return data


def describe_run(path: Path) -> tuple[str, str]:
    """Do not infer real hardware from directory name, recency or config defaults."""
    summary = read_mapping(path / "summary.json")
    config = read_mapping(path / "config_snapshot.yaml")
    mode = {"real": "真机", "simulation": "仿真", "simulation_preview": "仿真预演"}.get(summary.get("mode"), "未知")
    strategy = "未知"
    if "continuous_provenance" in config or "continuous_simulation" in config:
        strategy = "连续"
    elif (path / "simulation_config_snapshot.yaml").is_file():
        read_mapping(path / "simulation_config_snapshot.yaml")
        strategy, mode = "原离散", "仿真"
    elif (path / "samples.csv").is_file():
        # Schema identifies the legacy logger, but cannot establish hardware mode.
        with (path / "samples.csv").open(encoding="utf-8", newline="") as handle:
            fields = next(csv.reader(handle), [])
        if "commanded_speed_mps" in fields and "force_reference" not in fields:
            strategy = "原离散"
    return strategy, mode


def choose_run() -> Path:
    candidates = set()
    for base in (ROOT / "data", ROOT / "simulation_outputs"):
        if base.is_dir():
            for marker in ("summary.json", "samples.csv", "simulation_log.csv"):
                candidates.update(p.parent.resolve() for p in base.rglob(marker))
    ordered = sorted(candidates, key=str)
    for index, path in enumerate(ordered, 1):
        try:
            strategy, mode = describe_run(path)
        except (ValueError, OSError) as exc:
            strategy, mode = "未知", f"元数据错误：{exc}"
        print(f"{index}. [{strategy} / {mode}] {path}")
    if not ordered:
        print("默认目录没有运行记录；可输入实际运行目录完整路径。")
    while True:
        value = ask("选择上列序号或输入运行目录完整路径，无默认最新目录")
        if value.isdigit() and 1 <= int(value) <= len(ordered):
            return ordered[int(value)-1]
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = ROOT / path
        if value and path.is_dir():
            return path.resolve()
        print("无效目录或序号。")


def require_files(path: Path, names: tuple[str, ...]) -> None:
    for name in names:
        if not (path / name).is_file():
            raise ValueError(f"回放缺文件：{path / name}")
    sample = path / names[0]
    if sample.suffix == ".csv":
        with sample.open(encoding="utf-8", newline="") as handle:
            rows = csv.DictReader(handle)
            if next(rows, None) is None:
                raise ValueError(f"回放没有采样数据：{sample}")


def build_command(step: str) -> list[str]:
    command = list(STEPS[step][1])
    if step == "14":
        path = choose_path("箱体标定文件", ROOT / "workspace/config/workspace_calibration.yaml")
        return command + ["--calibration", str(path)]
    if step in {"15", "16"}:
        scene = choose_path("场景 YAML 文件", ROOT / "simulation/scene_continuous.yaml")
        # Validate using the same offline loader as the child, before launch.
        from simulation.simulator import load_simulation_config
        load_simulation_config(scene)
        command += ["--scene", str(scene), "--output", str(ROOT / "simulation_outputs/continuous_preview")]
        if step == "16":
            while True:
                value = ask("模拟时长（秒，有限正数；回车 12；只缩短原预算）") or "12"
                try:
                    duration = float(value)
                    if math.isfinite(duration) and duration > 0:
                        break
                except ValueError:
                    pass
                print("时长必须是有限正数。")
            command += ["--duration", str(duration)]
    if step in {"19", "20"}:
        path = choose_run()
        strategy, mode = describe_run(path)
        print(f"所选运行目录：{path}\n策略：{strategy}；设备模式：{mode}")
        for name in ("summary.json", "termination.json"):
            if (path / name).is_file():
                print(f"{name}:\n" + json.dumps(read_mapping(path / name), ensure_ascii=False, indent=2))
        if step == "19":
            if strategy == "连续":
                require_files(path, ("samples.csv", "config_snapshot.yaml", "policy_waypoints.csv"))
                return [sys.executable, "tools/visualize_continuous_run.py", str(path), "--format", "none"]
            if strategy == "原离散" and (path / "samples.csv").is_file():
                require_files(path, ("samples.csv", "config_snapshot.yaml", "boundary_points.csv", "policy_waypoints.csv"))
                if not any((path / name).is_file() for name in ("boundary_recovery_rays.csv", "corner_search_rays.csv")):
                    raise ValueError("回放缺文件：boundary_recovery_rays.csv 或 corner_search_rays.csv")
                return [sys.executable, "tools/visualize_run.py", "--run-dir", str(path)]
            if strategy != "原离散":
                raise ValueError("无法识别日志策略，未启动回放；请检查元数据和采样文件。")
            print("原离散 simulation_log 使用已有箱体坐标可视化工具，需要匹配的四角标定。")
        sample = "samples.csv" if (path / "samples.csv").is_file() else "simulation_log.csv"
        require_files(path, (sample,))
        calibration = choose_path("匹配本轮的箱体标定文件", path / "workspace_calibration_snapshot.yaml"
                                  if (path / "workspace_calibration_snapshot.yaml").is_file()
                                  else ROOT / "workspace/config/workspace_calibration.yaml")
        output = path / ("workspace_view_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        return [sys.executable, "visualize_workspace.py", "--run-dir", str(path),
                "--calibration", str(calibration), "--output-dir", str(output)]
    return command


def run_child(command: list[str]) -> int:
    # Inherit terminal and process group: START echo and child Q/Esc handling stay intact.
    # subprocess.run's interrupt cleanup can kill the child before its safety cleanup.
    child = subprocess.Popen(command, cwd=ROOT, env=clean_child_environment())
    interrupted = False
    while True:
        try:
            code = child.wait()
            return 130 if interrupted else code
        except KeyboardInterrupt:
            interrupted = True
            print("\n已收到 Ctrl+C；等待子程序停止和安全收尾，期间不能启动下一任务。", flush=True)
            # Terminal SIGINT reaches both processes. Do not send a duplicate interrupt
            # into the child's cleanup, kill it, or return to the menu while it lives.


def run_step(step: str) -> int:
    if step not in STEPS:
        print("输入无效，请输入 0～20。")
        return 2
    name = STEPS[step][0]
    try:
        dependency_error = check_dependency(step)
        if dependency_error:
            raise ValueError(dependency_error)
        if step in MOTION_CONFIRMATIONS:
            confirmation = MOTION_CONFIRMATIONS[step]
            print(f"\n警告：{name}。可能连接控制接口并发送真机运动命令。")
            print("须先检查现场、急停和完整返回/扫描路径；保留子程序 START/返回确认，不自动重试。")
            print("连续：Q/Esc/Ctrl+C 原地停止。原离散：Q 按原配置返回，Esc/Ctrl+C 不返回。")
            if ask(f"输入 {confirmation} 继续") != confirmation:
                raise Cancelled
        if step == "9":
            print("读取当前 active TCP 并自动保存到 config.yaml 的 tcp.offset；保留原配置备份。")
            print("请先在示教器标定并启用正确针尖 TCP；本工具不调用 setTcp，也不能代替针尖标定。")
        command = build_command(step)
        print(f"\n开始：{name}\n执行：{shlex.join(command)}", flush=True)
        result = run_child(command)
    except (Cancelled, EOFError):
        print("已取消，未启动子程序。")
        return 130
    except KeyboardInterrupt:
        print("已中断，未启动子程序。")
        return 130
    except Exception as exc:
        print(f"不能完成：{type(exc).__name__}: {exc}")
        return 2
    if result == 0:
        print("子程序已结束（退出码 0）；取消、停止或保存不代表扫描/几何/真机验证成功。请查看实际结果与停止原因。")
    else:
        print(f"子程序失败或中断（退出码 {result}）；请保留完整错误，不自动重试或进入下一任务。")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description="UR7e + PX6D 中文分阶段入口"
    )
    parser.add_argument("--step", choices=sorted(STEPS), help="直接执行指定步骤")
    args = parser.parse_args()
    if args.step:
        return run_step(args.step)

    while True:
        print_menu()
        try:
            selected = input("请输入数字并按回车：").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n已退出。")
            return 0
        if selected == "0":
            print("已退出。")
            return 0
        run_step(selected)
        try:
            input("\n按回车返回菜单……")
        except (EOFError, KeyboardInterrupt):
            print("\n已退出。")
            return 130


if __name__ == "__main__":
    raise SystemExit(main())
