"""Run the original Arm Console interface as a ROS 2 client."""
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions

from .gui_session import GuiSession, install_session


def main(args=None):
    # Keep the context alive long enough to request Disconnect on a window close
    # or launch shutdown; Tk's event loop does not process ROS signal callbacks.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    session = None
    app = None
    stop_requested = False
    def request_stop(signum, frame):
        nonlocal stop_requested
        if app is None:
            raise KeyboardInterrupt
        stop_requested = True
    previous = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        session = GuiSession('/dum_e')
        install_session(session)
        from .gui_app import ArmConsole
        app = ArmConsole()
        def check_shutdown():
            if stop_requested or not rclpy.ok():
                app.destroy()
            else:
                app.after(100, check_shutdown)
        app.after(100, check_shutdown)
        app.mainloop()
    except KeyboardInterrupt:
        if app is not None:
            app.destroy()
    except ValueError as error:
        print(f'ROS GUI: {error}')
        return 1
    finally:
        if session is not None:
            session.disable_serial()
            session.destroy_node()
        install_session(None)
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return 0
