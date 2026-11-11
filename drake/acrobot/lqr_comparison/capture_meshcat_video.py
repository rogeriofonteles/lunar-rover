"""Capture the saved Drake/Meshcat animation frame by frame using Chrome.

Requires the Python playwright package and system Google Chrome and ffmpeg.
First run record_drake_acrobot.py in the project's Drake container.
"""

from pathlib import Path
import subprocess

from playwright.sync_api import sync_playwright


ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "acrobot_video"
OUTPUT = ROOT / "acrobot_drake_simulation.mp4"
FPS = 30


def main():
    with sync_playwright() as p:
        browser = p.chromium.launch(
            executable_path="/usr/bin/google-chrome", headless=True,
            args=["--no-sandbox", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
        )
        page = browser.new_page(viewport={"width": 1600, "height": 900}, device_scale_factor=1)
        page.goto((ASSETS / "drake_meshcat.html").as_uri())
        page.wait_for_function("window.viewer && viewer.animator.duration >= 10")
        page.evaluate("""() => {
            viewer.animator.pause();
            viewer.gui.domElement.style.display = 'none';
            const style = document.createElement('style');
            style.textContent = `#stats-plot, #status-message {display:none!important}
                .caption {position:fixed;left:54px;z-index:100;color:#24354c;
                          font-family:Arial,sans-serif;pointer-events:none}
                #heading {top:36px;font-size:30px;font-weight:600}
                #detail {top:80px;font-size:17px;color:#53657b}
                #clock {bottom:43px;font-size:22px}
                #label {right:54px;left:auto;bottom:43px;font-size:18px}`;
            document.head.appendChild(style);
            for (const [id,text] of Object.entries({
                heading:'Drake acrobot simulation',
                detail:'Receding-horizon LQR  ·  N = 100  ·  Control interval = 0.10 s',
                clock:'', label:'Initial position'
            })) {
                const d=document.createElement('div');
                d.id=id;d.className='caption';d.textContent=text;
                document.body.appendChild(d);
            }
        }""")
        encoder = subprocess.Popen([
            "ffmpeg", "-hide_banner", "-loglevel", "warning", "-y",
            "-f", "image2pipe", "-framerate", str(FPS), "-vcodec", "png", "-i", "pipe:0",
            "-an", "-c:v", "libx264", "-threads", "4", "-crf", "18",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(OUTPUT),
        ], stdin=subprocess.PIPE)
        try:
            indices = [0] * FPS + list(range(10 * FPS + 1)) + [10 * FPS] * (2 * FPS)
            previous, png = None, None
            for index in indices:
                if index != previous:
                    page.evaluate("""t => {
                        viewer.animator.seek(t);
                        document.getElementById('clock').textContent = `Simulation time: ${t.toFixed(2)} s`;
                        document.getElementById('label').textContent =
                            t === 0 ? 'Initial position' : t >= 8 ? 'Upright equilibrium' : 'Receding-horizon control';
                        viewer.render();
                    }""", index / FPS)
                    png = page.screenshot(type="png")
                    if index in (0, 30, 300):
                        (ASSETS / f"drake_frame_{index:03d}.png").write_bytes(png)
                    previous = index
                encoder.stdin.write(png)
            encoder.stdin.close()
            if encoder.wait() != 0:
                raise RuntimeError("Video encoder failed")
        finally:
            if encoder.poll() is None:
                encoder.kill()
                encoder.wait()
            browser.close()
    print(OUTPUT)


if __name__ == "__main__":
    main()
