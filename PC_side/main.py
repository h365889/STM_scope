import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import serial
import serial.tools.list_ports
import numpy as np

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

DEFAULT_SAMPLES = 1024
DEFAULT_SAMPLE_RATE = 50000
SCOPE_BLOCK_BYTES = 256
SERIAL_BAUDRATE = 115200
MAX_PLOT_POINTS = 8192


class STM32Gui:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 mérési beállítások")
        self.root.geometry("1180x720")
        self.root.minsize(1000, 650)

        self.ser = None
        self.rx_stop = threading.Event()
        self.rx_queue = queue.Queue()
        self.rx_thread = None
        self.serial_lock = threading.Lock()

        self.mode = "scope"
        self.scope_buffer = bytearray()
        self.trigg_buffer = bytearray()
        self.trigg_expected_bytes = 0

        self.raw_data =np.empty(0, dtype=np.float64)
        self.plot_data = np.empty(0, dtype=np.float64)

        self.last_plot_time = 0.0
        self.updating = False

        self.samples_var = tk.IntVar(value=DEFAULT_SAMPLES)
        self.rate_var = tk.IntVar(value=DEFAULT_SAMPLE_RATE)
        self.time_var = tk.DoubleVar(
            value=DEFAULT_SAMPLES / DEFAULT_SAMPLE_RATE * 1000.0
        )
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Kapcsolat nincs ellenőrizve.")
        self.mode_var = tk.StringVar(value="Aktuális mód: nincs kiválasztva")


        self.samples_var.trace_add("write", self.samples_changed)
        self.rate_var.trace_add("write", self.rate_changed)
        self.time_var.trace_add("write", self.time_changed)

        self.build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(20, self.process_rx_queue)
        self.root.after(50, self.update_plot)


    def build_ui(self):
        main = ttk.Frame(self.root, padding=15)
        main.pack(fill="both", expand=True)

        controls = ttk.Frame(main, width=330)
        controls.pack(side="left", fill="y", padx=(0, 15))
        controls.pack_propagate(False)

        plot_frame = ttk.LabelFrame(main, text="ADC jel", padding=8)
        plot_frame.pack(side="right", fill="both", expand=True)

        ttk.Label(controls, text="Mérési paraméterek",
                  font=("Arial", 15, "bold")).pack(pady=10)

        ttk.Label(controls, text="Mintaszám [db]").pack(anchor="w")
        ttk.Scale(controls, from_=64, to=4096, variable=self.samples_var,
                  command=self.samples_changed).pack(fill="x")
        ttk.Entry(controls, textvariable=self.samples_var).pack(fill="x", pady=4)

        ttk.Label(controls, text="Mintavételi frekvencia [Hz]").pack(anchor="w")
        ttk.Scale(controls, from_=15, to=100000, variable=self.rate_var,
                  command=self.rate_changed).pack(fill="x")
        ttk.Entry(controls, textvariable=self.rate_var).pack(fill="x", pady=4)

        ttk.Label(controls, text="Mérési idő [ms]").pack(anchor="w")
        ttk.Scale(controls, from_=1, to=10000, variable=self.time_var,
                  command=self.time_changed).pack(fill="x")
        ttk.Entry(controls, textvariable=self.time_var).pack(fill="x", pady=4)

        ttk.Separator(controls).pack(fill="x", pady=10)
        ttk.Label(controls, text="USB COM port:").pack(anchor="w")
        port_row = ttk.Frame(controls)
        port_row.pack(fill="x", pady=4)
        self.port_box = ttk.Combobox(port_row, textvariable=self.port_var,
                                     state="readonly", width=13)
        self.port_box.pack(side="left", fill="x", expand=True)
        ttk.Button(port_row, text="Frissítés",
                   command=self.refresh_ports).pack(side="left", padx=5)

        self.connect_button = ttk.Button(controls, text="Kapcsolódás",
                                         command=self.toggle_connection)
        self.connect_button.pack(fill="x", pady=(5, 10))

        ttk.Button(controls, text="Paraméterek küldése",
                   command=self.send_parameters).pack(fill="x", pady=3)

        ttk.Separator(controls).pack(fill="x", pady=10)
        ttk.Label(controls, text="Üzemmód").pack(anchor="w")
        ttk.Button(controls, text="Scope mód",
                   command=self.set_scope_mode).pack(fill="x", pady=3)
        ttk.Button(controls, text="Triggerelt mérés mód",
                   command=self.set_measurement_mode).pack(fill="x", pady=3)
        ttk.Button(controls, text="Trigger",
                   command=self.send_trigger).pack(fill="x", pady=3)

        ttk.Separator(controls).pack(fill="x", pady=10)
        ttk.Label(controls, textvariable=self.mode_var,
                  font=("Arial", 10, "bold")).pack(anchor="w", pady=3)
        ttk.Label(controls, textvariable=self.status_var,
                  wraplength=310).pack(anchor="w", pady=5)

        self.figure = Figure(figsize=(8, 5), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("ADC adatok")
        self.ax.set_xlabel("Minta index")
        self.ax.set_ylabel("Feszültség [V]")
        self.ax.grid(True)
        self.line, = self.ax.plot([], [], linewidth=1)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def set_vars(self, samples=None, rate=None, duration_ms=None):
        self.updating = True
        try:
            if samples is not None:
                self.samples_var.set(int(samples))
            if rate is not None:
                self.rate_var.set(int(rate))
            if duration_ms is not None:
                self.time_var.set(float(duration_ms))
        finally:
            self.updating = False

    def calculate_time(self):
        if self.updating:
            return
        try:
            n = int(self.samples_var.get())
            fs = float(self.rate_var.get())
            if n > 0 and fs > 0:
                self.set_vars(duration_ms=n / fs * 1000.0)
        except (ValueError, tk.TclError):
            pass

    def calculate_rate(self):
        if self.updating:
            return
        try:
            n = int(self.samples_var.get())
            t_ms = float(self.time_var.get())
            if n > 0 and t_ms > 0:
                self.set_vars(rate=round(n / (t_ms / 1000.0)))
        except (ValueError, tk.TclError):
            pass

    def samples_changed(self, _value=None):
        if self.updating:
            return
        try:
            n = max(64, min(4096, round(float(self.samples_var.get()))))
            self.set_vars(samples=n)
            self.calculate_time()
        except (ValueError, tk.TclError):
            pass

    def rate_changed(self, _value=None):
        if self.updating:
            return
        try:
            fs = max(15, min(100000, round(float(self.rate_var.get()))))
            self.set_vars(rate=fs)
            self.calculate_time()
        except (ValueError, tk.TclError):
            pass

    def time_changed(self, _value=None):
        if self.updating:
            return
        try:
            t_ms = max(1.0, min(10000.0, float(self.time_var.get())))
            self.set_vars(duration_ms=round(t_ms, 2))
            self.calculate_rate()
        except (ValueError, tk.TclError):
            pass

    def get_parameters(self):
        n = int(self.samples_var.get())
        fs = int(self.rate_var.get())
        if not 64 <= n <= 4096:
            raise ValueError("A mintaszám 64 és 4096 közé essen.")
        if not 15 <= fs <= 100000:
            raise ValueError("A mintavételi frekvencia 15 és 100000 Hz közé essen.")
        return n, fs


    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("USB hiba", "Nincs kiválasztott COM port.")
            return
        try:
            self.ser = serial.Serial(port, SERIAL_BAUDRATE,
                                     timeout=0.05, write_timeout=1)
            self.ser.reset_input_buffer()
            self.rx_stop.clear()
            self.rx_thread = threading.Thread(target=self.reader_worker,
                                              daemon=True)
            self.rx_thread.start()
            self.connect_button.configure(text="Kapcsolat bontása")
            self.status_var.set(f"Kapcsolódva: {port}")
        except serial.SerialException as exc:
            self.ser = None
            messagebox.showerror("USB kommunikációs hiba", str(exc))

    def disconnect(self):
        self.rx_stop.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.3)
        try:
            if self.ser:
                self.ser.close()
        except serial.SerialException:
            pass
        self.ser = None
        self.connect_button.configure(text="Kapcsolódás")
        self.status_var.set("Kapcsolat bontva.")

    def reader_worker(self):
        while not self.rx_stop.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    break
                data = self.ser.read(1024)
                if data:
                    self.rx_queue.put(data)
            except (serial.SerialException, OSError) as exc:
                self.rx_queue.put(("ERROR", str(exc)))
                break

    def write_command(self, command):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Nincs kapcsolat",
                                   "Előbb kapcsolódj az STM32-höz.")
            return False
        try:
            with self.serial_lock:
                self.ser.write((command + "\n").encode("ascii"))
                self.ser.flush()
            return True
        except (serial.SerialException, OSError) as exc:
            messagebox.showerror("USB kommunikációs hiba", str(exc))
            return False

    def clear_rx_buffers(self):
        self.scope_buffer.clear()
        self.trigg_buffer.clear()
        while True:
            try:
                self.rx_queue.get_nowait()
            except queue.Empty:
                break
        try:
            if self.ser and self.ser.is_open:
                self.ser.reset_input_buffer()
        except serial.SerialException:
            pass


    def send_parameters(self):
        try:
            n, fs = self.get_parameters()
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("Hibás paraméter", str(exc))
            return
        if self.write_command(f"SET,{n},{fs}"):
            self.status_var.set(f"Paraméterek elküldve: N={n}, fs={fs} Hz")

    def set_scope_mode(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Nincs kapcsolat", "Előbb kapcsolódj az STM32-höz.")
            return
        self.clear_rx_buffers()
        self.mode = "scope"
        self.trigg_expected_bytes = 0
        if self.write_command("SCOPE"):
            self.mode_var.set("Aktuális mód: Scope")
            self.status_var.set("Scope mód: nyers 256 bájtos blokkok fogadása.")

    def set_measurement_mode(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Nincs kapcsolat", "Előbb kapcsolódj az STM32-höz.")
            return
        try:
            n, fs = self.get_parameters()
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("Hibás paraméter", str(exc))
            return

        self.clear_rx_buffers()
        self.mode = "measurement"
        self.trigg_expected_bytes = 0
        if not self.write_command(f"SET,{n},{fs}"):
            return
        if self.write_command("TRIGG"):
            self.mode_var.set("Aktuális mód: Triggerelt mérés")
            self.status_var.set(f"{n} minta, {fs} Hz – triggerre vár.")

    def send_trigger(self):
        if self.mode != "measurement":
            messagebox.showwarning("Nem triggerelt mód",
                                   "Előbb válaszd ki a Triggerelt mérés módot.")
            return
        try:
            n, _fs = self.get_parameters()
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("Hibás paraméter", str(exc))
            return

        self.trigg_buffer.clear()
        self.trigg_expected_bytes = n * 2
        if self.write_command("T"):
            self.status_var.set(f"Trigger elküldve – {n} minta fogadása...")


    def process_rx_queue(self):
        try:
            while True:
                item = self.rx_queue.get_nowait()
                if isinstance(item, tuple) and item[0] == "ERROR":
                    self.status_var.set("Soros olvasási hiba: " + item[1])
                    continue
                if self.mode == "scope":
                    self.scope_buffer.extend(item)
                    self.process_scope_bytes()
                elif self.mode == "measurement" and self.trigg_expected_bytes:
                    self.process_measurement_bytes(item)
        except queue.Empty:
            pass
        self.root.after(20, self.process_rx_queue)

    def process_scope_bytes(self):
        while len(self.scope_buffer) >= SCOPE_BLOCK_BYTES:
            block = bytes(self.scope_buffer[:SCOPE_BLOCK_BYTES])
            del self.scope_buffer[:SCOPE_BLOCK_BYTES]
            samples = np.frombuffer(block, dtype="<u2")
            self.append_scope_samples(samples)
            self.plot_data = (self.raw_data*3.3)/4096

    def process_measurement_bytes(self, data):
        remaining = self.trigg_expected_bytes - len(self.trigg_buffer)
        if remaining <= 0:
            return
        self.trigg_buffer.extend(data[:remaining])

        if len(self.trigg_buffer) == self.trigg_expected_bytes:
            samples = np.frombuffer(bytes(self.trigg_buffer), dtype="<u2").copy()
            self.trigg_expected_bytes = 0
            self.trigg_buffer.clear()
            self.raw_data = samples.astype(np.float64)
            self.plot_data = (self.raw_data * 3.3) / 4096
            self.mode_var.set("Aktuális mód: Triggerelt mérés – kész")
            self.status_var.set(f"Mérés kész: {len(samples)} minta megérkezett.")

    def append_scope_samples(self, samples):
        values = samples.astype(np.float64)
        if self.raw_data.size == 0:
            self.raw_data = values[-MAX_PLOT_POINTS:]
        else:
            self.raw_data = np.concatenate((self.raw_data, values))[-MAX_PLOT_POINTS:]

    def update_plot(self):
        if self.plot_data.size:
            x = np.arange(self.plot_data.size)
            self.line.set_data(x, self.plot_data)
            self.ax.set_xlim(0, max(1, self.plot_data.size - 1))
            self.ax.set_ylim(bottom=-0.7, top=4)
            self.canvas.draw_idle()
        self.root.after(50, self.update_plot)

    def on_close(self):
        self.rx_stop.set()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except serial.SerialException:
            pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = STM32Gui(root)
    root.mainloop()
