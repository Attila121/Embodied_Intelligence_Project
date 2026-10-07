#!/usr/bin/env python3
"""Hold-to-drive keyboard window for the TurtleBot3 simulation."""

LINEAR_SPEED = 0.15  # metres/second
TURN_SPEED = 0.7  # radians/second
DIRECTIONS = {"w", "a", "s", "d"}


class Keys:
    def __init__(self):
        self.held = set()
        self.braking = False

    def press(self, key):
        if key in DIRECTIONS:
            self.held.add(key)
        elif key in {"space", "escape"}:
            self.braking = True

    def release(self, key):
        self.held.discard(key)
        if not self.held:
            self.braking = False

    def clear(self):
        self.held.clear()
        self.braking = False

    def velocity(self):
        if self.braking:
            return 0.0, 0.0
        forward = int("w" in self.held) - int("s" in self.held)
        turn = int("a" in self.held) - int("d" in self.held)
        return forward * LINEAR_SPEED, turn * TURN_SPEED


def main():
    import signal
    import time
    import tkinter as tk
    from tkinter import ttk

    import rclpy
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("game_keyboard_control")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    keys = Keys()
    pending_releases = {}
    root = None

    def publish():
        msg = Twist()
        msg.linear.x, msg.angular.z = keys.velocity()
        publisher.publish(msg)
        return msg

    try:
        root = tk.Tk()
        root.title("TurtleBot3 — hold to drive")
        root.resizable(False, False)
        panel = ttk.Frame(root, padding=24)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="Hold to drive", font=("Arial", 20, "bold")).pack()
        ttk.Label(panel, text="W  forward    S  backward\nA  turn left    D  turn right",
                  justify="center", font=("Arial", 14)).pack(pady=15)
        ttk.Label(panel, text="Release keys to stop. Combine W + A/D to steer.\n"
                  "Space / Esc: stop until movement keys are released.\n"
                  "Keep this window focused. Switching windows stops motion.",
                  justify="center").pack()
        status = tk.StringVar(value="Stopped — click this window to control")
        ttk.Label(panel, textvariable=status).pack(pady=(16, 0))

        def cancel_releases():
            for timer in pending_releases.values():
                root.after_cancel(timer)
            pending_releases.clear()

        def pressed(event):
            key = event.keysym.lower()
            timer = pending_releases.pop(key, None)
            if timer is not None:
                root.after_cancel(timer)
            keys.press(key)
            publish()

        def released(event):
            key = event.keysym.lower()

            def commit_release():
                pending_releases.pop(key, None)
                keys.release(key)
                publish()

            # X11 key repeat generates release/press pairs. The next press
            # cancels this idle callback; a real release stops without waiting
            # for a keyboard-repeat timeout.
            timer = pending_releases.pop(key, None)
            if timer is not None:
                root.after_cancel(timer)
            pending_releases[key] = root.after_idle(commit_release)

        def unfocused(event):
            cancel_releases()
            keys.clear()
            publish()
            status.set("Stopped — click this window to control")

        def tick():
            if not rclpy.ok():
                root.quit()
                return
            msg = publish()
            status.set(f"Forward {msg.linear.x:+.2f} m/s   Turn {msg.angular.z:+.2f} rad/s")
            root.after(50, tick)

        root.bind("<KeyPress>", pressed)
        root.bind("<KeyRelease>", released)
        root.bind("<FocusOut>", unfocused)
        root.protocol("WM_DELETE_WINDOW", root.quit)
        signal.signal(signal.SIGINT, lambda *_: root.quit())
        signal.signal(signal.SIGTERM, lambda *_: root.quit())
        root.after(50, tick)
        root.mainloop()
    finally:
        keys.clear()
        if rclpy.ok():
            # Give DDS time to transmit the final zero before destroying it.
            for _ in range(3):
                publish()
                time.sleep(0.05)
        if root is not None:
            root.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
