"""Native viewer startup and orderly OpenGL shutdown for pinned MuJoCo 3.3.7."""
from contextlib import contextmanager
import threading


@contextmanager
def native_viewer(model, data, maximize=True, show_ui=False):
    import glfw
    from mujoco import viewer as mujoco_viewer

    if not glfw.init():
        raise ValueError("Could not initialize the viewer display")
    glfw.window_hint(glfw.MAXIMIZED, glfw.TRUE if maximize else glfw.FALSE)
    previous_threads = set(threading.enumerate())
    handle = mujoco_viewer.launch_passive(
        model, data, show_left_ui=show_ui, show_right_ui=show_ui)
    # In the pinned binding, close() requests exit but does not join its daemon
    # rendering thread. Wait for that specific thread before Python/GLFW teardown;
    # otherwise large shadow/frame buffers can still be freeing at process exit.
    render_threads = [thread for thread in threading.enumerate()
                      if thread not in previous_threads
                      and getattr(thread, "_target", None) is mujoco_viewer._launch_internal]
    try:
        yield handle
    finally:
        handle.close()
        for thread in render_threads:
            thread.join(timeout=10)
            if thread.is_alive():
                raise RuntimeError("MuJoCo's rendering thread did not finish shutting down")
