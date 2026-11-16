"""
sim_gui.pyw - Window for the flight replay simulator (double-click to open).

Pick a flight's nav CSV, press Run: the simulator is rebuilt if its settings or code changed, runs,
and writes the output CSV. Same as run_sim.ps1, without the command line.
Settings: sim/config.h (flight software) and sim/sim_settings.h (sim only).
"""
import glob
import json
import os
import queue
import shutil
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
FW_DIR = os.path.dirname(SIM_DIR)
BUILD_DIR = os.path.join(SIM_DIR, "build")
EXE = os.path.join(BUILD_DIR, "Release", "navsim.exe")
STATE_FILE = os.path.join(BUILD_DIR, "gui_state.json")   # remembers the last files picked
VIEWER_LOG = os.path.join(BUILD_DIR, "viewer_errors.txt")
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)
# The flight data viewer in this repo: AFSElec/CSV Viewer/flight_viewer.py (Main is AFSElec/Code/MainSoftware/Main)
VIEWER = os.path.normpath(os.path.join(FW_DIR, "..", "..", "..", "CSV Viewer", "flight_viewer.py"))


def gui_python():
    """pythonw.exe next to the running Python (no console window), else the running Python."""
    exe = sys.executable
    w = os.path.join(os.path.dirname(exe), "pythonw.exe")
    return w if os.path.isfile(w) else exe


def find_cmake():
    found = shutil.which("cmake")
    if found:
        return found
    candidates = [r"C:\Program Files\CMake\bin\cmake.exe"]
    candidates += sorted(glob.glob(os.path.expanduser(r"~\.pico-sdk\cmake\*\bin\cmake.exe")), reverse=True)
    return next((c for c in candidates if os.path.isfile(c)), None)


def default_output(path):
    root, _ = os.path.splitext(path)
    return root + "_sim.csv"


class SimGui:
    def __init__(self, root):
        self.root = root
        self.lines = queue.Queue()
        self.running = False
        root.title("Flight replay simulator")
        root.geometry("860x600")
        root.minsize(640, 420)

        self.input_var = tk.StringVar()
        self.output_var = tk.StringVar()
        self.reset_var = tk.BooleanVar(value=False)
        self.compare_var = tk.BooleanVar(value=False)

        top = ttk.Frame(root, padding=10)
        top.pack(fill="x")
        top.columnconfigure(1, weight=1)

        ttk.Label(top, text="Flight CSV (nav_*.csv)").grid(row=0, column=0, sticky="w", pady=3)
        ttk.Entry(top, textvariable=self.input_var).grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(top, text="Browse...", command=self.pick_input).grid(row=0, column=2)

        ttk.Label(top, text="Output CSV").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Entry(top, textvariable=self.output_var).grid(row=1, column=1, sticky="ew", padx=6)
        ttk.Button(top, text="Browse...", command=self.pick_output).grid(row=1, column=2)

        ttk.Checkbutton(top, text="Reset sim settings to the firmware's config.h before running",
                        variable=self.reset_var).grid(row=2, column=1, sticky="w", pady=(4, 0))

        buttons = ttk.Frame(root, padding=(10, 0))
        buttons.pack(fill="x")
        self.run_btn = ttk.Button(buttons, text="Run simulation", command=self.run)
        self.run_btn.pack(side="left")
        ttk.Button(buttons, text="Edit config.h",
                   command=lambda: self.edit(os.path.join(SIM_DIR, "config.h"))).pack(side="left", padx=(12, 4))
        ttk.Button(buttons, text="Edit sim_settings.h",
                   command=lambda: self.edit(os.path.join(SIM_DIR, "sim_settings.h"))).pack(side="left")
        self.folder_btn = ttk.Button(buttons, text="Show output in folder", command=self.show_output,
                                     state="disabled")
        self.folder_btn.pack(side="right")
        self.open_btn = ttk.Button(buttons, text="Open output", command=self.open_output, state="disabled")
        self.open_btn.pack(side="right", padx=4)

        viewer_row = ttk.Frame(root, padding=(10, 6, 10, 0))
        viewer_row.pack(fill="x")
        self.viewer_btn = ttk.Button(viewer_row, text="Open output in CSV Viewer", command=self.open_viewer,
                                     state="disabled")
        self.viewer_btn.pack(side="left")
        ttk.Checkbutton(viewer_row, text="also open the original flight (to compare)",
                        variable=self.compare_var).pack(side="left", padx=8)

        self.status = ttk.Label(root, text="Pick a flight CSV and press Run.", padding=(10, 6))
        self.status.pack(fill="x")

        log_frame = ttk.Frame(root, padding=(10, 0, 10, 10))
        log_frame.pack(fill="both", expand=True)
        self.log = tk.Text(log_frame, wrap="none", font=("Consolas", 10), state="disabled")
        yscroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        xscroll = ttk.Scrollbar(log_frame, orient="horizontal", command=self.log.xview)
        self.log.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        self.log.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        self.log.tag_configure("error", foreground="#c62828")
        self.log.tag_configure("warn", foreground="#b26a00")
        self.log.tag_configure("event", foreground="#1565c0")
        self.log.tag_configure("head", font=("Consolas", 10, "bold"))

        self.load_state()
        root.after(100, self.drain)

    # ---------- files ----------
    def pick_input(self):
        start = os.path.dirname(self.input_var.get()) or os.path.expanduser("~")
        path = filedialog.askopenfilename(title="Flight nav CSV", initialdir=start,
                                          filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if path:
            path = os.path.normpath(path)
            self.input_var.set(path)
            self.output_var.set(default_output(path))

    def pick_output(self):
        current = self.output_var.get() or (default_output(self.input_var.get()) if self.input_var.get() else "")
        path = filedialog.asksaveasfilename(title="Save simulated CSV as", defaultextension=".csv",
                                            initialdir=os.path.dirname(current) or None,
                                            initialfile=os.path.basename(current) or None,
                                            filetypes=[("CSV files", "*.csv")])
        if path:
            self.output_var.set(os.path.normpath(path))

    def edit(self, path):
        code = shutil.which("code")
        if code:
            subprocess.Popen([code, path], creationflags=NO_WINDOW)
        else:
            subprocess.Popen(["notepad.exe", path])

    def open_output(self):
        path = self.output_var.get()
        if os.path.isfile(path):
            os.startfile(path)

    def show_output(self):
        path = self.output_var.get()
        if os.path.isfile(path):
            subprocess.Popen(["explorer", "/select,", path])

    def open_viewer(self):
        out = self.output_var.get()
        if not os.path.isfile(out):
            messagebox.showerror("Simulator", "No output CSV yet: run the simulation first.")
            return
        if not os.path.isfile(VIEWER):
            messagebox.showerror("Simulator", "CSV Viewer not found at\n" + VIEWER)
            return
        files = [out]
        inp = self.input_var.get()
        if self.compare_var.get() and os.path.isfile(inp):
            files.insert(0, inp)   # same format, so the viewer can overlay them (Compare mode)
        try:
            os.makedirs(BUILD_DIR, exist_ok=True)
            err = open(VIEWER_LOG, "w")
            proc = subprocess.Popen([gui_python(), VIEWER] + files, cwd=os.path.dirname(VIEWER),
                                    stdout=subprocess.DEVNULL, stderr=err, creationflags=NO_WINDOW)
        except OSError as e:
            messagebox.showerror("Simulator", "Couldn't start the CSV Viewer:\n" + str(e))
            return
        self.status.configure(text="Opening the CSV Viewer...")
        self.root.after(4000, lambda: self.check_viewer(proc, err))

    def check_viewer(self, proc, err):
        """If the viewer quit straight away (e.g. a missing package), show why."""
        code = proc.poll()
        if code is None or code == 0:
            return
        err.close()
        try:
            with open(VIEWER_LOG, encoding="utf-8", errors="replace") as fp:
                tail = fp.read()[-1500:]
        except OSError:
            tail = ""
        hint = ("\n\nInstall its packages with:\npip install -r \"" +
                os.path.join(os.path.dirname(VIEWER), "requirements.txt") + "\"") if "ModuleNotFound" in tail else ""
        messagebox.showerror("Simulator", "The CSV Viewer closed with an error:\n\n" + tail + hint)

    def load_state(self):
        try:
            with open(STATE_FILE) as fp:
                s = json.load(fp)
            self.input_var.set(s.get("input", ""))
            self.output_var.set(s.get("output", ""))
        except (OSError, ValueError):
            pass
        if os.path.isfile(self.output_var.get()):   # output from an earlier run: can be opened right away
            for b in (self.open_btn, self.folder_btn, self.viewer_btn):
                b.configure(state="normal")

    def save_state(self):
        try:
            os.makedirs(BUILD_DIR, exist_ok=True)
            with open(STATE_FILE, "w") as fp:
                json.dump({"input": self.input_var.get(), "output": self.output_var.get()}, fp)
        except OSError:
            pass

    # ---------- log ----------
    def write(self, text, tag=None):
        self.lines.put((text, tag))

    def drain(self):
        try:
            while True:
                item = self.lines.get_nowait()
                if item[0] is None:   # worker finished: (None, (ok, message))
                    self.finish(*item[1])
                    continue
                text, tag = item
                if tag is None:
                    low = text.lower()
                    if "error" in low or "can't" in low or "failed" in low:
                        tag = "error"
                    elif "warning" in low:
                        tag = "warn"
                    elif "STATE ->" in text or "ROLL TEST ->" in text:
                        tag = "event"
                    elif text.startswith(("Original flight", "Simulation:")):
                        tag = "head"
                self.log.configure(state="normal")
                self.log.insert("end", text + "\n", tag)
                self.log.see("end")
                self.log.configure(state="disabled")
        except queue.Empty:
            pass
        self.root.after(100, self.drain)

    # ---------- run ----------
    def run(self):
        if self.running:
            return
        inp = self.input_var.get().strip().strip('"')
        out = self.output_var.get().strip().strip('"') or default_output(inp)
        if not inp or not os.path.isfile(inp):
            messagebox.showerror("Simulator", "Pick an existing flight CSV first.")
            return
        if os.path.normcase(os.path.abspath(inp)) == os.path.normcase(os.path.abspath(out)):
            messagebox.showerror("Simulator", "The output file can't be the input file.")
            return
        if self.reset_var.get() and not messagebox.askyesno(
                "Simulator", "Replace sim/config.h with the firmware's config.h?\nYour sim setting changes will be lost."):
            return
        cmake = find_cmake()
        if not cmake:
            messagebox.showerror("Simulator", "CMake not found. Install it or add it to PATH.")
            return

        self.output_var.set(out)
        self.save_state()
        self.running = True
        self.run_btn.configure(state="disabled")
        self.open_btn.configure(state="disabled")
        self.folder_btn.configure(state="disabled")
        self.viewer_btn.configure(state="disabled")
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")
        self.status.configure(text="Running...")
        threading.Thread(target=self.worker, args=(cmake, inp, out, self.reset_var.get()), daemon=True).start()

    def stream(self, cmd, show=True):
        """Runs cmd, sending its output to the log. Returns the exit code."""
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                encoding="utf-8", errors="replace", creationflags=NO_WINDOW)
        for line in proc.stdout:
            line = line.rstrip()
            if show and line:
                self.write(line)
        return proc.wait()

    def worker(self, cmake, inp, out, reset):
        try:
            if reset:
                shutil.copyfile(os.path.join(FW_DIR, "config.h"), os.path.join(SIM_DIR, "config.h"))
                self.write("sim/config.h replaced with the firmware's config.h", "warn")

            self.write("Building the simulator (only what changed)...", "head")
            if not os.path.isfile(os.path.join(BUILD_DIR, "CMakeCache.txt")):
                if self.stream([cmake, "-S", SIM_DIR, "-B", BUILD_DIR, "-G", "Visual Studio 17 2022", "-A", "x64"]):
                    raise RuntimeError("CMake configure failed (needs Visual Studio 2022 with the C++ workload)")
            if self.stream([cmake, "--build", BUILD_DIR, "--config", "Release", "--",
                            "/nologo", "/verbosity:minimal"]):
                raise RuntimeError("Build failed: see the errors above (often a typo in config.h or sim_settings.h)")

            self.write("")
            self.write("Running the simulation...", "head")
            if self.stream([EXE, inp, out]):
                raise RuntimeError("The simulation stopped with an error (see above)")
            self.lines.put((None, (True, "Done: " + out)))
        except Exception as e:   # report anything in the window rather than dying silently
            self.write(str(e), "error")
            self.lines.put((None, (False, "Failed: " + str(e))))

    def finish(self, ok, message):
        self.running = False
        self.run_btn.configure(state="normal")
        self.status.configure(text=message)
        if ok:
            self.open_btn.configure(state="normal")
            self.folder_btn.configure(state="normal")
            self.viewer_btn.configure(state="normal")


if __name__ == "__main__":
    root = tk.Tk()
    SimGui(root)
    root.mainloop()
