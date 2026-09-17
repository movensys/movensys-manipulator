#!/usr/bin/env python3
"""Camera transform tuner with GUI sliders for nvblox camera TF."""

import math
import threading
import tkinter as tk
from tkinter import ttk

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


TRANSFORM_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


class CameraTransformTuner(Node):
    def __init__(self):
        super().__init__("camera_transform_tuner")

        self.declare_parameter("parent_frame", "world_manipulator")
        self.declare_parameter("camera_count", 2)
        self.declare_parameter("tf_time_offset", 0.0)

        self.parent_frame = self.get_parameter("parent_frame").value
        self.camera_count = int(self.get_parameter("camera_count").value)
        self.camera_count = max(1, self.camera_count)
        self.tf_time_offset = float(self.get_parameter("tf_time_offset").value)

        self.camera_names = []
        self.frames = []
        self.defaults = []
        for index in range(self.camera_count):
            prefix = f"camera_{index}"
            self.declare_parameter(f"{prefix}_name", f"camera {index}")
            self.declare_parameter(f"{prefix}_frame", f"camera_nvblox_{index}_link")
            self.declare_parameter(f"{prefix}_x", -0.7 if index == 0 else 0.7)
            self.declare_parameter(f"{prefix}_y", -0.25)
            self.declare_parameter(f"{prefix}_z", 0.87)
            self.declare_parameter(f"{prefix}_roll", -0.055834655172414045)
            self.declare_parameter(f"{prefix}_pitch", 0.9366455172413797)
            self.declare_parameter(f"{prefix}_yaw", 0.4757999999999999)

            self.camera_names.append(self.get_parameter(f"{prefix}_name").value)
            self.frames.append(self.get_parameter(f"{prefix}_frame").value)
            self.defaults.append({
                key: float(self.get_parameter(f"{prefix}_{key}").value)
                for key in TRANSFORM_KEYS
            })

        self.transforms = [dict(values) for values in self.defaults]
        self.active_camera = 0
        self.tf_broadcaster = TransformBroadcaster(self)
        self._lock = threading.Lock()
        self._timer = self.create_timer(0.05, self.publish_transforms)

        self.get_logger().info("Camera Transform Tuner started")
        for index, frame in enumerate(self.frames):
            self.get_logger().info(
                f"camera_{index}: {self.parent_frame} -> {frame}, "
                f"initial={self.transforms[index]}"
            )
        if self.tf_time_offset != 0.0:
            self.get_logger().info(f"TF time offset: {self.tf_time_offset:.3f} sec")

    def publish_transforms(self):
        with self._lock:
            transforms = [dict(values) for values in self.transforms]

        stamp = (
            self.get_clock().now() + Duration(seconds=self.tf_time_offset)
        ).to_msg()
        messages = []
        for index, values in enumerate(transforms):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.frames[index]
            t.transform.translation.x = values["x"]
            t.transform.translation.y = values["y"]
            t.transform.translation.z = values["z"]

            qx, qy, qz, qw = self.euler_to_quaternion(
                values["roll"], values["pitch"], values["yaw"]
            )
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            messages.append(t)

        self.tf_broadcaster.sendTransform(messages)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def set_active_camera(self, index):
        with self._lock:
            self.active_camera = index

    def get_transform(self, index):
        with self._lock:
            return dict(self.transforms[index])

    def update_active_transform(self, values):
        with self._lock:
            self.transforms[self.active_camera].update(values)

    def reset_active_transform(self):
        with self._lock:
            self.transforms[self.active_camera] = dict(self.defaults[self.active_camera])
            return dict(self.transforms[self.active_camera])


class TunerGUI:
    def __init__(self, node: CameraTransformTuner):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Camera Transform Tuner")
        self.root.geometry("720x620")

        self.saved_positions = [None for _ in range(node.camera_count)]
        self.step_pos = tk.StringVar(value="0.01")
        self.step_rot = tk.StringVar(value="0.01")
        self.sliders = {}

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        title = ttk.Label(main_frame, text="Camera Transform Tuner",
                          font=("Helvetica", 14, "bold"))
        title.grid(row=0, column=0, columnspan=5, pady=10)

        selector_frame = ttk.Frame(main_frame)
        selector_frame.grid(row=1, column=0, columnspan=5, pady=5, sticky=(tk.W, tk.E))
        ttk.Label(selector_frame, text="Camera:").grid(row=0, column=0, padx=5)
        self.camera_selector = ttk.Combobox(
            selector_frame,
            state="readonly",
            width=28,
            values=[
                f"camera_{index}: {node.frames[index]}"
                for index in range(node.camera_count)
            ],
        )
        self.camera_selector.current(0)
        self.camera_selector.grid(row=0, column=1, padx=5)
        self.camera_selector.bind("<<ComboboxSelected>>", self.on_camera_selected)

        self.frame_info = ttk.Label(
            main_frame,
            text=f"{node.parent_frame} -> {node.frames[0]}"
        )
        self.frame_info.grid(row=2, column=0, columnspan=5, pady=5)

        step_frame = ttk.Frame(main_frame)
        step_frame.grid(row=3, column=0, columnspan=5, pady=5)

        ttk.Label(step_frame, text="Step size - Position:").grid(row=0, column=0, padx=5)
        ttk.Entry(step_frame, textvariable=self.step_pos, width=8).grid(row=0, column=1, padx=5)

        ttk.Label(step_frame, text="Rotation:").grid(row=0, column=2, padx=5)
        ttk.Entry(step_frame, textvariable=self.step_rot, width=8).grid(row=0, column=3, padx=5)

        initial = node.get_transform(0)
        slider_configs = [
            ("x", -3.0, 3.0, initial["x"], "m", False),
            ("y", -3.0, 3.0, initial["y"], "m", False),
            ("z", -1.0, 3.0, initial["z"], "m", False),
            ("roll", -3.14159, 3.14159, initial["roll"], "rad", True),
            ("pitch", -3.14159, 3.14159, initial["pitch"], "rad", True),
            ("yaw", -3.14159, 3.14159, initial["yaw"], "rad", True),
        ]

        for i, (name, min_val, max_val, value, unit, is_rot) in enumerate(slider_configs):
            row = i + 4
            ttk.Label(main_frame, text=f"{name} ({unit}):", width=10).grid(
                row=row, column=0, sticky=tk.W, pady=3
            )
            ttk.Button(
                main_frame, text="-", width=3,
                command=lambda n=name, r=is_rot: self.on_step_minus(n, r)
            ).grid(row=row, column=1, sticky=tk.E, pady=3)

            var = tk.DoubleVar(value=value)
            slider = ttk.Scale(
                main_frame, from_=min_val, to=max_val, variable=var,
                orient=tk.HORIZONTAL, length=260,
                command=lambda _value, n=name: self.on_slider_change(n)
            )
            slider.grid(row=row, column=2, sticky=(tk.W, tk.E), pady=3, padx=2)

            ttk.Button(
                main_frame, text="+", width=3,
                command=lambda n=name, r=is_rot: self.on_step_plus(n, r)
            ).grid(row=row, column=3, sticky=tk.W, pady=3)

            entry_var = tk.StringVar(value=f"{value:.4f}")
            entry = ttk.Entry(main_frame, textvariable=entry_var, width=12)
            entry.grid(row=row, column=4, sticky=tk.E, pady=3, padx=5)
            entry.bind("<Return>", lambda _event, n=name: self.on_entry_change(n))
            entry.bind("<FocusOut>", lambda _event, n=name: self.on_entry_change(n))

            self.sliders[name] = {
                "var": var,
                "entry_var": entry_var,
                "min": min_val,
                "max": max_val,
                "is_rotation": is_rot,
            }

        button_frame1 = ttk.Frame(main_frame)
        button_frame1.grid(row=10, column=0, columnspan=5, pady=5)

        ttk.Button(button_frame1, text="Reset Camera", command=self.reset_values).grid(
            row=0, column=0, padx=5
        )
        ttk.Button(button_frame1, text="Print Current", command=self.print_current_values).grid(
            row=0, column=1, padx=5
        )
        ttk.Button(button_frame1, text="Print All", command=self.print_all_values).grid(
            row=0, column=2, padx=5
        )

        button_frame2 = ttk.Frame(main_frame)
        button_frame2.grid(row=11, column=0, columnspan=5, pady=5)

        ttk.Button(button_frame2, text="Save Camera", command=self.save_position).grid(
            row=0, column=0, padx=5
        )
        self.load_btn = ttk.Button(
            button_frame2, text="Load Saved", command=self.load_saved_position,
            state=tk.DISABLED
        )
        self.load_btn.grid(row=0, column=1, padx=5)

        self.saved_label = ttk.Label(main_frame, text="No position saved", foreground="gray")
        self.saved_label.grid(row=12, column=0, columnspan=5, pady=5)

        self.output_text = tk.Text(main_frame, height=6, width=82)
        self.output_text.grid(row=13, column=0, columnspan=5, pady=10)

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(2, weight=1)

    def on_camera_selected(self, _event=None):
        index = self.camera_selector.current()
        self.node.set_active_camera(index)
        self.frame_info.config(text=f"{self.node.parent_frame} -> {self.node.frames[index]}")
        self.set_sliders(self.node.get_transform(index))
        self.update_saved_label()

    def set_sliders(self, values):
        for name, value in values.items():
            self.sliders[name]["var"].set(value)
            self.sliders[name]["entry_var"].set(f"{value:.4f}")

    def slider_values(self):
        return {
            name: self.sliders[name]["var"].get()
            for name in TRANSFORM_KEYS
        }

    def on_slider_change(self, name):
        value = self.sliders[name]["var"].get()
        self.sliders[name]["entry_var"].set(f"{value:.4f}")
        self.node.update_active_transform(self.slider_values())

    def on_entry_change(self, name):
        try:
            value = float(self.sliders[name]["entry_var"].get())
            min_val = self.sliders[name]["min"]
            max_val = self.sliders[name]["max"]
            value = max(min_val, min(max_val, value))
            self.sliders[name]["var"].set(value)
            self.sliders[name]["entry_var"].set(f"{value:.4f}")
            self.node.update_active_transform(self.slider_values())
        except ValueError:
            value = self.sliders[name]["var"].get()
            self.sliders[name]["entry_var"].set(f"{value:.4f}")

    def get_step_size(self, is_rotation):
        try:
            return float(self.step_rot.get() if is_rotation else self.step_pos.get())
        except ValueError:
            return 0.01

    def on_step_plus(self, name, is_rotation):
        self.step_value(name, self.get_step_size(is_rotation))

    def on_step_minus(self, name, is_rotation):
        self.step_value(name, -self.get_step_size(is_rotation))

    def step_value(self, name, step):
        current = self.sliders[name]["var"].get()
        new_val = current + step
        new_val = max(self.sliders[name]["min"], min(self.sliders[name]["max"], new_val))
        self.sliders[name]["var"].set(new_val)
        self.sliders[name]["entry_var"].set(f"{new_val:.4f}")
        self.node.update_active_transform(self.slider_values())

    def reset_values(self):
        values = self.node.reset_active_transform()
        self.set_sliders(values)

    def save_position(self):
        index = self.camera_selector.current()
        self.saved_positions[index] = self.slider_values()
        self.update_saved_label()
        self.node.get_logger().info(f"Saved camera_{index}: {self.saved_positions[index]}")

    def load_saved_position(self):
        index = self.camera_selector.current()
        if self.saved_positions[index] is None:
            return
        self.set_sliders(self.saved_positions[index])
        self.node.update_active_transform(self.saved_positions[index])
        self.node.get_logger().info(f"Loaded saved camera_{index}")

    def update_saved_label(self):
        index = self.camera_selector.current()
        saved = self.saved_positions[index]
        if saved is None:
            self.saved_label.config(text="No position saved", foreground="gray")
            self.load_btn.config(state=tk.DISABLED)
            return

        self.saved_label.config(
            text=f"Saved camera_{index}: x={saved['x']:.3f}, y={saved['y']:.3f}, z={saved['z']:.3f}",
            foreground="green"
        )
        self.load_btn.config(state=tk.NORMAL)

    def format_launch_args(self, index, values):
        return (
            f"camera_{index}_x:={values['x']:.6f} "
            f"camera_{index}_y:={values['y']:.6f} "
            f"camera_{index}_z:={values['z']:.6f} "
            f"camera_{index}_roll:={values['roll']:.6f} "
            f"camera_{index}_pitch:={values['pitch']:.6f} "
            f"camera_{index}_yaw:={values['yaw']:.6f}"
        )

    def print_current_values(self):
        index = self.camera_selector.current()
        output = self.format_launch_args(index, self.slider_values())
        self.output_text.delete(1.0, tk.END)
        self.output_text.insert(tk.END, output)
        self.node.get_logger().info(output)

    def print_all_values(self):
        lines = []
        for index in range(self.node.camera_count):
            lines.append(self.format_launch_args(index, self.node.get_transform(index)))
        output = " \\\n".join(lines)
        self.output_text.delete(1.0, tk.END)
        self.output_text.insert(tk.END, output)
        self.node.get_logger().info(output.replace("\n", " "))

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTransformTuner()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gui = TunerGUI(node)
    gui.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
