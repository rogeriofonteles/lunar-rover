"""Report the actual OpenGL renderer, including display-free EGL diagnostics."""
import argparse
import json
import os


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("glfw", "egl", "osmesa"), default="glfw")
    parser.add_argument("--require-nvidia", action="store_true")
    args = parser.parse_args()
    # Choose the backend before MuJoCo/PyOpenGL initializes a platform.
    os.environ["MUJOCO_GL"] = args.backend
    import mujoco
    from OpenGL import GL

    context = mujoco.GLContext(64, 64)
    try:
        context.make_current()
        def gl_string(token):
            value = GL.glGetString(token)
            if value is None:
                raise RuntimeError("No current OpenGL context")
            return value.decode()
        info = {"backend": args.backend,
                "vendor": gl_string(GL.GL_VENDOR),
                "renderer": gl_string(GL.GL_RENDERER),
                "version": gl_string(GL.GL_VERSION)}
        print(json.dumps(info, indent=2))
        if args.require_nvidia and "NVIDIA" not in info["vendor"]:
            raise SystemExit("Expected NVIDIA hardware rendering; check the container GPU configuration")
    finally:
        context.free()


if __name__ == "__main__":
    main()
