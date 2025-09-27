import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import queue

class MotorControllerGUI:
    """
    A Tkinter-based GUI to control a motor connected to a Raspberry Pi Pico
    via a serial (UART) connection, with a live output log and RPM display.
    """
    def __init__(self, root):
        self.root = root
        self.root.title("Pico Motor Controller")
        self.root.geometry("450x600")
        self.root.minsize(450, 550)

        # Serial connection state
        self.serial_connection = None
        self.is_connected = False
        self.is_armed = False
        self.reader_thread = None
        self.serial_queue = queue.Queue()

        # --- Style Configuration ---
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TButton", padding=6, relief="flat", font=('Helvetica', 10))
        style.configure("TLabel", font=('Helvetica', 10))
        style.configure("TEntry", font=('Helvetica', 10))
        style.configure("Red.TButton", foreground="red", background="#ffdddd")
        style.configure("Green.TButton", foreground="green", background="#ddffdd")
        style.configure("RPM.TLabel", font=('Helvetica', 16, 'bold'), foreground="#007acc")


        # --- Main Frames ---
        connection_frame = ttk.LabelFrame(self.root, text="1. Connection", padding=10)
        connection_frame.pack(padx=10, pady=10, fill="x")

        control_frame = ttk.LabelFrame(self.root, text="2. Motor Controls", padding=10)
        control_frame.pack(padx=10, pady=10, fill="x")
        
        log_frame = ttk.LabelFrame(self.root, text="Serial Output Log", padding=10)
        log_frame.pack(padx=10, pady=10, fill="both", expand=True)

        # --- Connection Widgets ---
        self.port_var = tk.StringVar()
        self.port_menu = ttk.Combobox(connection_frame, textvariable=self.port_var, state="readonly", width=15)
        self.port_menu.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        refresh_button = ttk.Button(connection_frame, text="Refresh Ports", command=self.populate_ports)
        refresh_button.grid(row=0, column=1, padx=5, pady=5)
        self.connect_button = ttk.Button(connection_frame, text="Connect", command=self.toggle_connection, width=12)
        self.connect_button.grid(row=0, column=2, padx=5, pady=5)
        connection_frame.grid_columnconfigure(0, weight=1)
        
        # --- Control Widgets ---
        ttk.Label(control_frame, text="Target RPM:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.rpm_entry = ttk.Entry(control_frame, width=10)
        self.rpm_entry.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        self.rpm_entry.insert(0, "3000")

        ttk.Label(control_frame, text="Setup Time (s):").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.setup_time_entry = ttk.Entry(control_frame, width=10)
        self.setup_time_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        self.setup_time_entry.insert(0, "5")

        ttk.Label(control_frame, text="Hold Time (s):").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.hold_time_entry = ttk.Entry(control_frame, width=10)
        self.hold_time_entry.grid(row=2, column=1, padx=5, pady=5, sticky="w")
        self.hold_time_entry.insert(0, "10")

        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=4, column=0, columnspan=2, pady=10)
        self.arm_button = ttk.Button(button_frame, text="Arm ESC", command=self.arm_esc)
        self.arm_button.pack(side="left", padx=5)
        self.ramp_button = ttk.Button(button_frame, text="Start Ramp", command=self.start_ramp, style="Green.TButton")
        self.ramp_button.pack(side="left", padx=5)
        self.interrupt_button = ttk.Button(button_frame, text="Interrupt", command=self.send_interrupt, style="Red.TButton")
        self.interrupt_button.pack(side="left", padx=5)

        # --- Live RPM Display (NEW) ---
        self.rpm_display_var = tk.StringVar(value="RPM: --")
        rpm_label = ttk.Label(control_frame, textvariable=self.rpm_display_var, style="RPM.TLabel")
        rpm_label.grid(row=3, column=0, columnspan=2, pady=10)

        # --- Log Widgets ---
        self.log_widget = scrolledtext.ScrolledText(log_frame, height=10, wrap=tk.WORD, font=("Consolas", 9), state='disabled')
        self.log_widget.pack(fill="both", expand=True)

        # --- Status Bar ---
        self.status_var = tk.StringVar(value="Status: Disconnected")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor="w")
        status_bar.pack(side=tk.BOTTOM, fill="x")
        
        # --- Initial State ---
        self.populate_ports()
        self.update_ui_state()
        self.process_serial_queue()

    def update_log(self, message):
        self.log_widget.configure(state='normal')
        self.log_widget.insert(tk.END, message + '\n')
        self.log_widget.configure(state='disabled')
        self.log_widget.see(tk.END)

    def process_serial_queue(self):
        try:
            while True:
                message = self.serial_queue.get_nowait()
                
                # Check for RPM messages first
                if message.startswith("RPM:"):
                    rpm_value = message.split(':')[1]
                    self.rpm_display_var.set(f"Live RPM: {rpm_value}")
                else:
                    self.update_log(message) # Log other messages

                # Check for any message confirming an armed state
                if "ESC Armed" in message or "Returning to ARMED state" in message or \
                   "Interrupt received" in message or "Warn: Already armed" in message:
                    self.is_armed = True
                    self.rpm_display_var.set("RPM: --") # Reset display
                    self.update_ui_state()

        except queue.Empty:
            pass
        self.root.after(100, self.process_serial_queue)

    def read_from_serial(self):
        while self.is_connected:
            try:
                line = self.serial_connection.readline().decode('utf-8').strip()
                if line:
                    self.serial_queue.put(line)
            except (serial.SerialException, TypeError):
                self.serial_queue.put("--- Serial connection lost ---")
                self.is_connected = False
                self.root.after(0, self.disconnect_serial)
                break

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "No COM port selected.")
            return
        try:
            self.serial_connection = serial.Serial(port, 9600, timeout=1)
            self.is_connected = True
            self.is_armed = False
            self.status_var.set(f"Status: Connected to {port}")
            self.update_log(f"--- Successfully connected to {port} ---")
            self.reader_thread = threading.Thread(target=self.read_from_serial, daemon=True)
            self.reader_thread.start()
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", f"Failed to connect to {port}.\n{e}")
            return
        self.update_ui_state()

    def disconnect_serial(self):
        if self.serial_connection:
            self.is_connected = False
            self.serial_connection.close()
            self.serial_connection = None
        self.is_armed = False
        self.status_var.set("Status: Disconnected")
        self.update_log("--- Disconnected ---")
        self.rpm_display_var.set("RPM: --")
        self.update_ui_state()

    def populate_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_menu['values'] = ports
        if ports:
            self.port_var.set(ports[0])

    def update_ui_state(self):
        # This logic is now corrected
        if self.is_connected:
            self.connect_button.config(text="Disconnect")
            self.port_menu.config(state="disabled")
            # Arm button is active only when connected but not yet armed.
            self.arm_button.config(state="normal" if not self.is_armed else "disabled")
            # Ramp button is active only when armed.
            self.ramp_button.config(state="normal" if self.is_armed else "disabled")
            # Interrupt button is active whenever we are connected, to stop any process.
            self.interrupt_button.config(state="normal")
        else:
            # All controls are disabled if not connected.
            self.connect_button.config(text="Connect")
            self.port_menu.config(state="readonly")
            self.arm_button.config(state="disabled")
            self.ramp_button.config(state="disabled")
            self.interrupt_button.config(state="disabled")

    def send_command(self, command):
        if self.is_connected:
            self.serial_connection.write((command + '\n').encode('utf-8'))
        else:
            messagebox.showwarning("Warning", "Not connected to any device.")

    def arm_esc(self):
        self.send_command("arm")
        self.update_log("-> Sent 'arm' command")

    def start_ramp(self):
        try:
            rpm = int(self.rpm_entry.get())
            setup_s = float(self.setup_time_entry.get())
            hold_s = float(self.hold_time_entry.get())
            setup_ms = int(setup_s * 1000)
            hold_ms = int(hold_s * 1000)
            command = f"ramp({rpm},{setup_ms},{hold_ms})"
            self.send_command(command)
            self.update_log(f"-> Sent '{command}'")
            self.is_armed = False
            self.update_ui_state()
        except ValueError:
            messagebox.showerror("Input Error", "Inputs must be valid numbers.")

    def send_interrupt(self):
        self.send_command("interrupt")
        self.update_log("-> Sent 'interrupt' command")

if __name__ == "__main__":
    app_root = tk.Tk()
    gui = MotorControllerGUI(app_root)
    app_root.mainloop()


