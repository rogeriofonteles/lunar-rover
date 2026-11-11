"""Copy Python sources and type stubs from Docker for host-side editor analysis.

These files are an editor cache, not a runnable Python environment.
"""

import argparse
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile


EXPORT = r'''
from pathlib import Path
import sys
import tarfile

roots = []
for pattern in (
    "/usr/lib/python3/dist-packages",
    "/usr/local/lib/python*/dist-packages",
    "/usr/local/lib/python*/site-packages",
    "/opt/ros/*/lib/python*/site-packages",
    "/opt/ros/*/local/lib/python*/dist-packages",
    "/opt/drake-env/lib/python*/site-packages",
    "/opt/mujoco-env/lib/python*/site-packages",
):
    roots.extend(Path("/").glob(pattern.lstrip("/")))
if not roots:
    raise RuntimeError("No Python package directories found")
with tarfile.open(fileobj=sys.stdout.buffer, mode="w|") as archive:
    for root in roots:
        for path in sorted(root.rglob("*")):
            if path.is_file() and (path.suffix in (".py", ".pyi") or path.name == "py.typed"):
                archive.add(path, arcname=str(path.relative_to(root)), recursive=False)
'''


def sync(command, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        temporary = Path(temporary)
        archive_path = temporary / "packages.tar"
        with archive_path.open("wb") as output:
            subprocess.run(command, input=EXPORT.encode(), stdout=output, check=True)
        extracted = temporary / "packages"
        with tarfile.open(archive_path) as archive:
            archive.extractall(extracted, filter="data")
        if destination.exists():
            shutil.rmtree(destination)
        extracted.rename(destination)
    print(f"Updated {destination}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drake-container", default="lunar-rover-jazzy-drake")
    parser.add_argument("--mujoco-container", default="lunar-rover-jazzy-mujoco")
    parser.add_argument("--ros-image", default="lunar-rover-jazzy-ros:v0.0.3")
    parser.add_argument("--only", choices=("drake", "ros", "mujoco"))
    args = parser.parse_args()
    cache = Path(__file__).resolve().parents[1] / ".python-libs"
    if args.only == "mujoco":
        sync(["docker", "exec", "-i", args.mujoco_container, "python", "-"], cache / "mujoco")
    if args.only in (None, "drake"):
        sync(["docker", "exec", "-i", args.drake_container, "python", "-"], cache / "drake")
        # Drake's compiled visualization module adds Python exports at runtime.
        # Represent that module as a package so static analysis sees them too.
        drake = cache / "drake" / "pydrake"
        stub = drake / "visualization.pyi"
        extras = drake / "visualization" / "_visualization_extra.py"
        if stub.exists() and extras.exists():
            exports = (
                "\nfrom ._model_visualizer import ModelVisualizer as ModelVisualizer\n"
                "from ._triad import AddFrameTriadIllustration as AddFrameTriadIllustration\n"
                "from ._meldis import Meldis as Meldis\n"
                "from ._video import VideoWriter as VideoWriter\n"
                "from ._plotting import plot_sublevelset_expression as plot_sublevelset_expression\n"
                "from ._plotting import plot_sublevelset_quadratic as plot_sublevelset_quadratic\n"
            )
            (extras.parent / "__init__.pyi").write_text(stub.read_text() + exports)
            stub.unlink()
    if args.only in (None, "ros"):
        sync([
            "docker", "run", "--rm", "-i", "--network", "none",
            "--entrypoint", "python3", args.ros_image, "-",
        ], cache / "ros")


if __name__ == "__main__":
    main()
