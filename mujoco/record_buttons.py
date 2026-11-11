"""Record reproducible 1080p button demonstrations using EGL and FFmpeg.

Inside the MuJoCo container: python record_buttons.py
Motion plays at simulated real time, with one second before and two after.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from arm.buttons import BUTTON_NAMES, button_target
from arm.model import ArmModel
from arm.mpc import MPC_SOLVERS
from arm.simulate import load_config, run_simulation


def record(button, destination, width, height, fps, mode="infinite", solver=None, horizon=None):
    config = load_config(button=button, mode=mode, solver=solver, horizon=horizon)
    destination.mkdir(parents=True, exist_ok=True)
    path = destination / button
    states, _, summary = run_simulation(button=button, mode=config["mode"],
                                       solver=config["solver"], horizon=config["horizon"],
                                       headless=True, wait=False, output=path)
    summary["horizon"] = config["horizon"]
    controller_label = (f"MPC / {config['solver'].upper()} / N={config['horizon']}"
                        if config["mode"] == "mpc" else "LQR")
    model = ArmModel(config["timestep"], config["physics_timestep"], config["torque_limit"])
    data = mujoco.MjData(model.model)
    model.model.vis.global_.offwidth = width
    model.model.vis.global_.offheight = height
    duration = summary["simulated_seconds"]
    state_times = np.arange(len(states))*model.dt
    frame_times = np.r_[np.zeros(fps), np.arange(0, duration+1e-9, 1/fps),
                        np.full(2*fps, duration)]
    colors = {"red": "#ff6c62", "yellow": "#ffdf65", "green": "#62efab"}
    title_font = ImageFont.load_default(size=round(height*.033))
    text_font = ImageFont.load_default(size=round(height*.021))
    target = button_target(button)["position_m"]
    encoder = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
               "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
               "-r", str(fps), "-i", "pipe:0", "-an", "-c:v", "libx264",
               "-threads", "2", "-preset", "medium", "-crf", "18",
               "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path.with_suffix(".mp4"))]
    with subprocess.Popen(encoder, stdin=subprocess.PIPE) as process:
        try:
            with mujoco.Renderer(model.model, height=height, width=width) as renderer:
                for index, t in enumerate(frame_times):
                    data.qpos[:] = [np.interp(t, state_times, states[:, j]) for j in range(6)]
                    data.qvel[:] = 0
                    mujoco.mj_forward(model.model, data)
                    renderer.update_scene(data, camera="buttons")
                    frame = Image.fromarray(renderer.render())
                    draw = ImageDraw.Draw(frame)
                    x, y = round(width*.025), round(height*.035)
                    draw.rounded_rectangle((x-16, y-12, x+width*.48, y+height*.125),
                                           radius=12, fill="#13232e")
                    draw.text((x, y), f"REACH / {button.upper()} BUTTON", font=title_font,
                              fill=colors[button])
                    draw.text((x, y+height*.045),
                              f"Target: ({target[0]:+.2f}, {target[1]:+.2f}, {target[2]:.2f}) m",
                              font=text_font, fill="white")
                    draw.text((x, y+height*.079), f"{controller_label}  |  t = {t:.2f} s",
                              font=text_font, fill="#c4d0d8")
                    process.stdin.write(frame.tobytes())
                    if index == 0:
                        frame.save(destination / f"{button}-start.png")
                    if index == len(frame_times)-1:
                        frame.save(destination / f"{button}-final.png")
                    if index % fps == 0:
                        print(f"Recording {button}: {index}/{len(frame_times)} frames", flush=True)
        finally:
            process.stdin.close()
            code = process.wait()
        if code:
            raise RuntimeError(f"FFmpeg failed with exit code {code}")
    summary["video"] = {"file": str(path.with_suffix(".mp4")), "width": width,
                        "height": height, "fps": fps, "frames": len(frame_times),
                        "duration_seconds": len(frame_times)/fps,
                        "initial_hold_seconds": 1, "final_hold_seconds": 2}
    path.with_suffix(".json").write_text(json.dumps(summary, indent=2)+"\n")
    print(f"Saved {path.with_suffix('.mp4')}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--button", choices=BUTTON_NAMES)
    parser.add_argument("--mode", choices=("infinite", "mpc"), default="infinite")
    parser.add_argument("--solver", choices=MPC_SOLVERS)
    parser.add_argument("--horizon", type=int)
    parser.add_argument("--output-dir", type=Path, default=Path("output/button-videos"))
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    if args.width < 2 or args.height < 2 or args.width % 2 or args.height % 2 or args.fps < 1:
        parser.error("Video dimensions must be positive even integers; fps must be positive")
    if args.horizon is not None and args.horizon < 1:
        parser.error("horizon must be positive")
    if shutil.which("ffmpeg") is None:
        parser.error("FFmpeg is required; rebuild the MuJoCo image with its current Dockerfile")
    for button in (args.button,) if args.button else BUTTON_NAMES:
        record(button, args.output_dir, args.width, args.height, args.fps,
               args.mode, args.solver, args.horizon)


if __name__ == "__main__":
    main()
