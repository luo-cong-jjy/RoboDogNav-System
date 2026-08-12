"""Out-of-process MuJoCo viewer helper.

Keeping the viewer in a separate Python process avoids a common WSL/OpenGL
failure mode where the native MuJoCo viewer segfaults and kills the trainer.
"""

from __future__ import annotations

import multiprocessing as mp
import queue
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class ViewerProcess:
    process: mp.Process
    state_queue: Any
    stop_event: Any

    def sync(self, qpos: np.ndarray, qvel: np.ndarray) -> None:
        state = (np.asarray(qpos, dtype=np.float64), np.asarray(qvel, dtype=np.float64))
        try:
            self.state_queue.put_nowait(state)
        except queue.Full:
            pass

    def close(self) -> None:
        self.stop_event.set()
        self.process.join(timeout=2.0)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1.0)


def start_viewer_process(xml_path: str | Path, control_dt: float, real_time: bool = False) -> ViewerProcess:
    ctx = mp.get_context("spawn")
    state_queue = ctx.Queue(maxsize=2)
    stop_event = ctx.Event()
    process = ctx.Process(
        target=_viewer_main,
        args=(str(Path(xml_path).resolve()), state_queue, stop_event, float(control_dt), bool(real_time)),
        daemon=True,
    )
    process.start()
    return ViewerProcess(process=process, state_queue=state_queue, stop_event=stop_event)


def _viewer_main(xml_path: str, state_queue: Any, stop_event: Any, control_dt: float, real_time: bool) -> None:
    try:
        import mujoco
        import mujoco.viewer

        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while not stop_event.is_set() and viewer.is_running():
                latest_state = None
                try:
                    while True:
                        latest_state = state_queue.get_nowait()
                except queue.Empty:
                    pass

                if latest_state is not None:
                    qpos, qvel = latest_state
                    data.qpos[:] = qpos
                    data.qvel[:] = qvel
                    mujoco.mj_forward(model, data)
                    viewer.sync()

                time.sleep(control_dt if real_time else 0.005)
    except BaseException as exc:
        # A GUI failure should not kill the trainer process.  Printing here is
        # enough to tell the user to use scripts/play.py after training instead.
        print(f"[WARN] MuJoCo viewer process exited: {exc}", flush=True)

