# ======================================================================
# viewer_process.py —— 独立进程 MuJoCo 查看器辅助（中文注释版）
# 作用：把 MuJoCo 原生查看器放进独立的 Python 子进程运行，训练主进程只
#       通过队列推送状态、通过事件通知关闭。这可以避免 WSL/OpenGL 下
#       原生查看器段错误（segfault）导致训练进程一起崩溃的常见故障。
# 说明：本文件只新增中文注释，未改动任何原始代码
# ======================================================================

"""Out-of-process MuJoCo viewer helper.

Keeping the viewer in a separate Python process avoids a common WSL/OpenGL
failure mode where the native MuJoCo viewer segfaults and kills the trainer.
"""

from __future__ import annotations

# 多进程：创建子进程查看器
import multiprocessing as mp
# 队列：进程间传递状态
import queue
# 时间：控制刷新间隔
import time
# 数据类：封装查看器句柄
from dataclasses import dataclass
# 路径库
from pathlib import Path
# 任意类型标注
from typing import Any

# NumPy：状态数组
import numpy as np


# 查看器进程句柄：封装子进程、状态队列与停止事件
@dataclass
class ViewerProcess:
    # 子进程对象
    process: mp.Process
    # 状态队列（训练进程 -> 查看器进程）
    state_queue: Any
    # 停止事件（通知查看器退出）
    stop_event: Any

    # 同步一帧状态到查看器（非阻塞，队列满则丢帧保证实时性）
    def sync(self, qpos: np.ndarray, qvel: np.ndarray) -> None:
        # 打包位姿与速度
        state = (np.asarray(qpos, dtype=np.float64), np.asarray(qvel, dtype=np.float64))
        try:
            # 非阻塞放入队列（队列满直接丢弃旧帧）
            self.state_queue.put_nowait(state)
        except queue.Full:
            pass

    # 关闭查看器：先发停止事件，再等待/强制结束子进程
    def close(self) -> None:
        # 通知查看器退出
        self.stop_event.set()
        # 最多等 2 秒
        self.process.join(timeout=2.0)
        if self.process.is_alive():
            # 仍未退出则强制终止
            self.process.terminate()
            self.process.join(timeout=1.0)


# 启动查看器子进程（spawn 上下文，xml_path 为模型文件路径）
def start_viewer_process(xml_path: str | Path, control_dt: float, real_time: bool = False) -> ViewerProcess:
    # 使用 spawn 启动方式（跨平台更安全）
    ctx = mp.get_context("spawn")
    # 状态队列：最多缓存 2 帧
    state_queue = ctx.Queue(maxsize=2)
    # 停止事件
    stop_event = ctx.Event()
    # 创建子进程运行 _viewer_main
    process = ctx.Process(
        target=_viewer_main,
        args=(str(Path(xml_path).resolve()), state_queue, stop_event, float(control_dt), bool(real_time)),
        daemon=True,   # 守护进程：主进程退出时自动结束
    )
    # 启动子进程
    process.start()
    # 返回句柄
    return ViewerProcess(process=process, state_queue=state_queue, stop_event=stop_event)


# 查看器主循环（在子进程中执行）
def _viewer_main(xml_path: str, state_queue: Any, stop_event: Any, control_dt: float, real_time: bool) -> None:
    try:
        # 延迟导入 MuJoCo（避免无 GUI 环境报错）
        import mujoco
        import mujoco.viewer

        # 加载模型
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        # 以被动模式启动查看器
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # 循环直到收到停止事件或窗口关闭
            while not stop_event.is_set() and viewer.is_running():
                # 取出队列中最新的一帧状态
                latest_state = None
                try:
                    while True:
                        latest_state = state_queue.get_nowait()
                except queue.Empty:
                    pass

                if latest_state is not None:
                    # 用最新状态覆盖仿真数据并同步显示
                    qpos, qvel = latest_state
                    data.qpos[:] = qpos
                    data.qvel[:] = qvel
                    mujoco.mj_forward(model, data)
                    viewer.sync()

                # 控制刷新间隔：实时模式按控制步长睡眠，否则 5ms
                time.sleep(control_dt if real_time else 0.005)
    except BaseException as exc:
        # A GUI failure should not kill the trainer process.  Printing here is
        # enough to tell the user to use scripts/play.py after training instead.
        # 注释（原文）：GUI 失败不应杀死训练进程；这里打印提示即可，
        # 并告诉用户训练后可改用 scripts/play.py 回放
        print(f"[WARN] MuJoCo viewer process exited: {exc}", flush=True)
