"""
GUI Control for 6-Axis Robotic Arm
Provides a tkinter-based interface for controlling the robotic arm using the RobotArm class.
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
from robot_arm.common.arm.arm import RobotArm
from robot_arm.common.utils import setup_logging

logger = setup_logging()


class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("6-Axis Robotic Arm Control")
        self.root.geometry("800x700")

        self.arm = RobotArm()
        self.is_connected = False
        self.monitoring = False
        self.monitor_thread = None

        self.setup_ui()

    def setup_ui(self):
        """Set up the user interface components."""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Connection frame
        self.setup_connection_frame(main_frame)

        # Motor control frame
        self.setup_motor_control_frame(main_frame)

        # Preset positions frame
        self.setup_preset_frame(main_frame)

        # Status frame
        self.setup_status_frame(main_frame)

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

    def setup_connection_frame(self, parent):
        """Set up connection controls."""
        conn_frame = ttk.LabelFrame(parent, text="Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2,
                        sticky=(tk.W, tk.E), pady=(0, 10))

        self.connect_btn = ttk.Button(
            conn_frame, text="Connect", command=self.connect_arm)
        self.connect_btn.grid(row=0, column=0, padx=(0, 5))

        self.disconnect_btn = ttk.Button(
            conn_frame, text="Disconnect", command=self.disconnect_arm, state=tk.DISABLED)
        self.disconnect_btn.grid(row=0, column=1, padx=5)

        self.torque_enable_btn = ttk.Button(
            conn_frame, text="Enable Torque", command=self.enable_torque, state=tk.DISABLED)
        self.torque_enable_btn.grid(row=0, column=2, padx=5)

        self.torque_disable_btn = ttk.Button(
            conn_frame, text="Disable Torque", command=self.disable_torque, state=tk.DISABLED)
        self.torque_disable_btn.grid(row=0, column=3, padx=5)

        self.connection_status = ttk.Label(
            conn_frame, text="Status: Disconnected", foreground="red")
        self.connection_status.grid(row=0, column=4, padx=(10, 0))

    def setup_motor_control_frame(self, parent):
        """Set up individual motor controls."""
        motor_frame = ttk.LabelFrame(parent, text="Motor Control", padding="5")
        motor_frame.grid(row=1, column=0, columnspan=2,
                         sticky=(tk.W, tk.E), pady=(0, 10))

        # Headers
        ttk.Label(motor_frame, text="Motor").grid(row=0, column=0, padx=5)
        ttk.Label(motor_frame, text="Current Position").grid(
            row=0, column=1, padx=5)
        ttk.Label(motor_frame, text="Target Position").grid(
            row=0, column=2, padx=5)
        ttk.Label(motor_frame, text="Speed").grid(row=0, column=3, padx=5)
        ttk.Label(motor_frame, text="Actions").grid(row=0, column=4, padx=5)

        self.motor_controls = {}

        # Motor limits and home positions
        motor_limits = {1: (775, 3304), 2: (810, 3261), 3: (1043, 3204),
                        4: (702, 3481), 5: (117, 3563), 6: (1379, 3144)}
        home_positions = {1: 2127, 2: 2986, 3: 1236, 4: 3443, 5: 2005, 6: 3116}

        for motor_id in range(1, 7):
            row = motor_id
            min_pos, max_pos = motor_limits[motor_id]
            home_pos = home_positions[motor_id]

            # Motor ID label
            ttk.Label(motor_frame, text=f"Motor {motor_id}").grid(
                row=row, column=0, padx=5, pady=2)

            # Current position display
            current_pos_var = tk.StringVar(value="---")
            current_pos_label = ttk.Label(
                motor_frame, textvariable=current_pos_var)
            current_pos_label.grid(row=row, column=1, padx=5, pady=2)

            # Target position entry
            target_pos_var = tk.IntVar(value=home_pos)
            target_pos_scale = ttk.Scale(motor_frame, from_=min_pos, to=max_pos,
                                         variable=target_pos_var, orient=tk.HORIZONTAL, length=150,
                                         command=lambda val, mid=motor_id: self.on_slider_change(mid, val))
            target_pos_scale.grid(row=row, column=2, padx=5, pady=2)

            target_pos_entry = ttk.Entry(
                motor_frame, textvariable=target_pos_var, width=6)
            target_pos_entry.grid(row=row, column=2, padx=(220, 5), pady=2)

            # Speed entry
            speed_var = tk.IntVar(value=1000)
            speed_entry = ttk.Entry(
                motor_frame, textvariable=speed_var, width=6)
            speed_entry.grid(row=row, column=3, padx=5, pady=2)

            # Move and Stop buttons
            button_frame = ttk.Frame(motor_frame)
            button_frame.grid(row=row, column=4, padx=5, pady=2)

            move_btn = ttk.Button(button_frame, text="Move",
                                  command=lambda mid=motor_id: self.move_motor(mid), state=tk.DISABLED)
            move_btn.grid(row=0, column=0, padx=(0, 2))

            stop_btn = ttk.Button(button_frame, text="Stop",
                                  command=lambda mid=motor_id: self.stop_motor(mid), state=tk.DISABLED)
            stop_btn.grid(row=0, column=1, padx=2)

            home_btn = ttk.Button(button_frame, text="Home",
                                  command=lambda mid=motor_id: self.home_motor(mid), state=tk.DISABLED)
            home_btn.grid(row=0, column=2, padx=2)

            self.motor_controls[motor_id] = {
                'current_pos_var': current_pos_var,
                'target_pos_var': target_pos_var,
                'speed_var': speed_var,
                'move_btn': move_btn,
                'stop_btn': stop_btn,
                'home_btn': home_btn,
                'min_pos': min_pos,
                'max_pos': max_pos,
                'home_pos': home_pos
            }

    def setup_preset_frame(self, parent):
        """Set up preset position controls."""
        preset_frame = ttk.LabelFrame(
            parent, text="Preset Positions", padding="5")
        preset_frame.grid(row=2, column=0, columnspan=2,
                          sticky=(tk.W, tk.E), pady=(0, 10))

        self.home_all_btn = ttk.Button(preset_frame, text="Home All Motors",
                                       command=self.home_all_motors, state=tk.DISABLED)
        self.home_all_btn.grid(row=0, column=0, padx=5)

        self.center_all_btn = ttk.Button(preset_frame, text="Center All Motors",
                                         command=self.center_all_motors, state=tk.DISABLED)
        self.center_all_btn.grid(row=0, column=1, padx=5)

        self.move_all_btn = ttk.Button(preset_frame, text="Move All",
                                       command=self.move_all_motors, state=tk.DISABLED)
        self.move_all_btn.grid(row=0, column=2, padx=5)

        # Speed control for preset movements
        ttk.Label(preset_frame, text="Preset Speed:").grid(
            row=0, column=3, padx=(20, 5))
        self.preset_speed_var = tk.IntVar(value=1000)
        preset_speed_entry = ttk.Entry(
            preset_frame, textvariable=self.preset_speed_var, width=6)
        preset_speed_entry.grid(row=0, column=4, padx=5)

    def setup_status_frame(self, parent):
        """Set up status monitoring."""
        status_frame = ttk.LabelFrame(
            parent, text="Status & Logs", padding="5")
        status_frame.grid(row=3, column=0, columnspan=2, sticky=(
            tk.W, tk.E, tk.N, tk.S), pady=(0, 10))

        # Position monitoring toggle
        self.monitor_var = tk.BooleanVar()
        monitor_check = ttk.Checkbutton(status_frame, text="Monitor Positions",
                                        variable=self.monitor_var, command=self.toggle_monitoring)
        monitor_check.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # Auto-move toggle
        self.auto_move_var = tk.BooleanVar()
        auto_move_check = ttk.Checkbutton(status_frame, text="Auto Move on Slider Change",
                                          variable=self.auto_move_var)
        auto_move_check.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        # Log display
        self.log_text = scrolledtext.ScrolledText(
            status_frame, height=8, width=70)
        self.log_text.grid(row=1, column=0, columnspan=2,
                           padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))

        status_frame.columnconfigure(0, weight=1)
        status_frame.rowconfigure(1, weight=1)

    def connect_arm(self):
        """Connect to the robotic arm."""
        try:
            success = self.arm.connect()
            if success:
                self.is_connected = True
                self.connection_status.config(
                    text="Status: Connected", foreground="green")
                self.enable_controls()
                self.log_message("Successfully connected to robotic arm")
                # Automatically start monitoring positions
                self.monitor_var.set(True)
                self.start_monitoring()
            else:
                self.log_message("Failed to connect to robotic arm", "ERROR")
                messagebox.showerror("Connection Error",
                                     "Failed to connect to robotic arm")
        except Exception as e:
            self.log_message(f"Connection error: {str(e)}", "ERROR")
            messagebox.showerror("Connection Error",
                                 f"Error connecting to arm: {str(e)}")

    def disconnect_arm(self):
        """Disconnect from the robotic arm."""
        try:
            if self.monitoring:
                self.stop_monitoring()
                self.monitor_var.set(False)
            self.arm.disconnect()
            self.is_connected = False
            self.connection_status.config(
                text="Status: Disconnected", foreground="red")
            self.disable_controls()
            self.log_message("Disconnected from robotic arm")
        except Exception as e:
            self.log_message(f"Disconnect error: {str(e)}", "ERROR")

    def enable_torque(self):
        """Enable torque for all motors."""
        try:
            self.arm.enable_torque()
            self.log_message("Torque enabled for all motors")
        except Exception as e:
            self.log_message(f"Error enabling torque: {str(e)}", "ERROR")

    def disable_torque(self):
        """Disable torque for all motors."""
        try:
            self.arm.disable_torque()
            self.log_message("Torque disabled for all motors")
        except Exception as e:
            self.log_message(f"Error disabling torque: {str(e)}", "ERROR")

    def move_motor(self, motor_id):
        """Move a specific motor to target position."""
        try:
            controls = self.motor_controls[motor_id]
            target_pos = controls['target_pos_var'].get()
            speed = controls['speed_var'].get()

            # Validate position within limits
            if not (controls['min_pos'] <= target_pos <= controls['max_pos']):
                messagebox.showerror("Invalid Position",
                                     f"Position {target_pos} is outside limits ({controls['min_pos']}-{controls['max_pos']})")
                return

            self.arm.move(motor_id, target_pos, speed)
            self.log_message(
                f"Moving motor {motor_id} to position {target_pos} at speed {speed}")
        except Exception as e:
            self.log_message(
                f"Error moving motor {motor_id}: {str(e)}", "ERROR")

    def stop_motor(self, motor_id):
        """Stop a specific motor."""
        try:
            self.arm.stop(motor_id)
            self.log_message(f"Stopped motor {motor_id}")
        except Exception as e:
            self.log_message(
                f"Error stopping motor {motor_id}: {str(e)}", "ERROR")

    def home_motor(self, motor_id):
        """Move a specific motor to home position."""
        try:
            controls = self.motor_controls[motor_id]
            speed = controls['speed_var'].get()
            home_pos = controls['home_pos']

            self.arm.move(motor_id, home_pos, speed)
            controls['target_pos_var'].set(home_pos)
            self.log_message(
                f"Moving motor {motor_id} to home position {home_pos}")
        except Exception as e:
            self.log_message(
                f"Error homing motor {motor_id}: {str(e)}", "ERROR")

    def home_all_motors(self):
        """Move all motors to home position."""
        try:
            speed = self.preset_speed_var.get()
            self.arm.return_to_home(speed)

            # Update target position displays
            for motor_id, controls in self.motor_controls.items():
                controls['target_pos_var'].set(controls['home_pos'])

            self.log_message(
                f"Moving all motors to home position at speed {speed}")
        except Exception as e:
            self.log_message(f"Error homing all motors: {str(e)}", "ERROR")

    def center_all_motors(self):
        """Move all motors to center position."""
        try:
            speed = self.preset_speed_var.get()
            self.arm.move_all_to_center(speed)

            # Update target position displays to center
            for motor_id, controls in self.motor_controls.items():
                center_pos = (controls['min_pos'] + controls['max_pos']) // 2
                controls['target_pos_var'].set(center_pos)

            self.log_message(
                f"Moving all motors to center position at speed {speed}")
        except Exception as e:
            self.log_message(f"Error centering all motors: {str(e)}", "ERROR")

    def move_all_motors(self):
        """Move all motors to their current slider positions."""
        try:
            speed = self.preset_speed_var.get()
            positions = []

            # Collect all target positions and validate them
            for motor_id in range(1, 7):
                controls = self.motor_controls[motor_id]
                target_pos = controls['target_pos_var'].get()

                # Validate position within limits
                if not (controls['min_pos'] <= target_pos <= controls['max_pos']):
                    messagebox.showerror("Invalid Position",
                                         f"Motor {motor_id} position {target_pos} is outside limits ({controls['min_pos']}-{controls['max_pos']})")
                    return
                positions.append((motor_id, target_pos))

            # Move all motors
            for motor_id, target_pos in positions:
                self.arm.move(motor_id, target_pos, speed)

            positions_str = ", ".join(
                [f"M{mid}:{pos}" for mid, pos in positions])
            self.log_message(
                f"Moving all motors to slider positions at speed {speed}: {positions_str}")

        except Exception as e:
            self.log_message(f"Error moving all motors: {str(e)}", "ERROR")

    def on_slider_change(self, motor_id, value):
        """Handle slider value changes."""
        # Update the IntVar to display integer values
        int_value = int(float(value))
        self.motor_controls[motor_id]['target_pos_var'].set(int_value)

        # Auto-move if enabled and connected
        if self.auto_move_var.get() and self.is_connected:
            self.auto_move_motor(motor_id, int_value)

    def auto_move_motor(self, motor_id, position):
        """Automatically move a motor when slider changes."""
        try:
            controls = self.motor_controls[motor_id]
            speed = controls['speed_var'].get()

            # Validate position within limits (should already be valid from slider)
            if controls['min_pos'] <= position <= controls['max_pos']:
                self.arm.move(motor_id, position, speed)
                self.log_message(
                    f"Auto-moving motor {motor_id} to position {position}")

        except Exception as e:
            self.log_message(
                f"Error auto-moving motor {motor_id}: {str(e)}", "ERROR")

    def toggle_monitoring(self):
        """Toggle position monitoring."""
        if self.monitor_var.get() and self.is_connected:
            self.start_monitoring()
        else:
            self.stop_monitoring()

    def start_monitoring(self):
        """Start position monitoring thread."""
        if not self.monitoring:
            self.monitoring = True
            self.monitor_thread = threading.Thread(
                target=self.monitor_positions, daemon=True)
            self.monitor_thread.start()
            self.log_message("Started position monitoring")

    def stop_monitoring(self):
        """Stop position monitoring."""
        if self.monitoring:
            self.monitoring = False
            self.log_message("Stopped position monitoring")

    def monitor_positions(self):
        """Monitor motor positions in a separate thread."""
        while self.monitoring and self.is_connected:
            try:
                for motor_id in range(1, 7):
                    try:
                        position = self.arm.read_position(motor_id)
                        if position is not None:
                            self.motor_controls[motor_id]['current_pos_var'].set(
                                str(position))
                        else:
                            self.log_message(
                                f"Warning: Could not read position for motor {motor_id}", "WARNING")
                    except Exception as motor_error:
                        self.log_message(
                            f"Error reading position for motor {motor_id}: {str(motor_error)}", "ERROR")
                        self.motor_controls[motor_id]['current_pos_var'].set(
                            "ERR")
                time.sleep(0.5)  # Update every 500ms
            except Exception as e:
                self.log_message(
                    f"Error in position monitoring: {str(e)}", "ERROR")
                break

    def enable_controls(self):
        """Enable GUI controls when connected."""
        self.connect_btn.config(state=tk.DISABLED)
        self.disconnect_btn.config(state=tk.NORMAL)
        self.torque_enable_btn.config(state=tk.NORMAL)
        self.torque_disable_btn.config(state=tk.NORMAL)
        self.home_all_btn.config(state=tk.NORMAL)
        self.center_all_btn.config(state=tk.NORMAL)
        self.move_all_btn.config(state=tk.NORMAL)

        for controls in self.motor_controls.values():
            controls['move_btn'].config(state=tk.NORMAL)
            controls['stop_btn'].config(state=tk.NORMAL)
            controls['home_btn'].config(state=tk.NORMAL)

    def disable_controls(self):
        """Disable GUI controls when disconnected."""
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.torque_enable_btn.config(state=tk.DISABLED)
        self.torque_disable_btn.config(state=tk.DISABLED)
        self.home_all_btn.config(state=tk.DISABLED)
        self.center_all_btn.config(state=tk.DISABLED)
        self.move_all_btn.config(state=tk.DISABLED)

        for controls in self.motor_controls.values():
            controls['move_btn'].config(state=tk.DISABLED)
            controls['stop_btn'].config(state=tk.DISABLED)
            controls['home_btn'].config(state=tk.DISABLED)
            controls['current_pos_var'].set("---")

    def log_message(self, message, level="INFO"):
        """Add a message to the log display."""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}\n"

        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)

        # Log to file as well
        if level == "ERROR":
            logger.error(message)
        else:
            logger.info(message)

    def on_closing(self):
        """Handle application closing."""
        if self.is_connected:
            self.disconnect_arm()
        self.root.destroy()


def main():
    """Main function to run the GUI application."""
    root = tk.Tk()
    app = RobotArmGUI(root)

    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)

    # Start the GUI event loop
    root.mainloop()


if __name__ == "__main__":
    main()
