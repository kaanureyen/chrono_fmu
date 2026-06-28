import os
import sys
import tkinter as tk
from tkinter import ttk, messagebox
import threading

# Add src/road_generator to system path to import the generator
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROAD_GEN_DIR = os.path.join(SCRIPT_DIR, "src", "road_generator")
sys.path.append(ROAD_GEN_DIR)

try:
    from road_generator import generate_road_profile
except ImportError:
    messagebox.showerror("Import Error", f"Could not import generate_road_profile. Make sure you are in the chrono_fmus root folder.")
    sys.exit(1)

class RedirectText:
    """Helper to redirect stdout to Tkinter text widget."""
    def __init__(self, text_widget):
        self.text_widget = text_widget

    def write(self, string):
        self.text_widget.insert(tk.END, string)
        self.text_widget.see(tk.END)

    def flush(self):
        pass

class RoadGeneratorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Chrono Co-Simulation Road Profile Generator")
        self.root.geometry("640x800")
        self.root.configure(bg="#121214")
        self.root.resizable(False, False)

        # Fallback database styles to prevent white-on-white dropdowns/inputs
        self.root.option_add("*background", "#121214")
        self.root.option_add("*foreground", "#e4e4e7")
        self.root.option_add("*TCombobox*Listbox.background", "#1e1e24")
        self.root.option_add("*TCombobox*Listbox.foreground", "#e4e4e7")
        self.root.option_add("*TCombobox*Listbox.selectBackground", "#4f46e5")
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#ffffff")

        # Style configurations
        self.style = ttk.Style()
        self.style.theme_use("clam")

        # Dark theme styling
        self.style.configure(".", background="#121214", foreground="#e4e4e7")
        self.style.configure("TLabel", background="#121214", foreground="#e4e4e7", font=("Segoe UI", 10))
        self.style.configure("TEntry", fieldbackground="#1e1e24", foreground="#ffffff", insertcolor="#ffffff", bordercolor="#3f3f46")
        self.style.configure("TCombobox", fieldbackground="#1e1e24", foreground="#ffffff", selectbackground="#4f46e5")
        self.style.configure("TCheckbutton", background="#121214", foreground="#e4e4e7", font=("Segoe UI", 10))
        
        # Style mappings for entries and dropdowns
        self.style.map("TEntry",
                       foreground=[("active", "#ffffff"), ("disabled", "#71717a")],
                       fieldbackground=[("active", "#1e1e24"), ("disabled", "#121214")])
        self.style.map("TCombobox",
                       foreground=[("active", "#ffffff"), ("disabled", "#71717a")],
                       fieldbackground=[("active", "#1e1e24"), ("disabled", "#121214")])
        
        # Primary Accent Button
        self.style.configure("Accent.TButton", 
                             background="#4f46e5", 
                             foreground="#ffffff", 
                             font=("Segoe UI", 11, "bold"), 
                             borderwidth=0, 
                             focuscolor="none")
        self.style.map("Accent.TButton", 
                       background=[("active", "#6366f1"), ("pressed", "#4338ca")])

        self.create_widgets()

    def create_widgets(self):
        # Header Label
        header_frame = tk.Frame(self.root, bg="#1a1a1e", height=80)
        header_frame.pack(fill=tk.X)
        header_frame.pack_propagate(False)

        header_lbl = tk.Label(header_frame, 
                              text="ROAD PROFILE & PATH GENERATOR", 
                              fg="#6366f1", 
                              bg="#1a1a1e", 
                              font=("Segoe UI", 14, "bold"))
        header_lbl.pack(pady=15)
        
        sub_lbl = tk.Label(header_frame, 
                           text="Chrono Vehicle FMI Co-Simulation Suite", 
                           fg="#a1a1aa", 
                           bg="#1a1a1e", 
                           font=("Segoe UI", 9, "italic"))
        sub_lbl.pack()

        # Parameter Form Frame
        form_frame = tk.Frame(self.root, bg="#121214", padx=20, pady=20)
        form_frame.pack(fill=tk.BOTH)

        # Helper to add inputs
        def add_field(row, label_text, var_name, default_val, tooltip=""):
            lbl = ttk.Label(form_frame, text=label_text)
            lbl.grid(row=row, column=0, sticky=tk.W, pady=8)
            
            entry = ttk.Entry(form_frame, font=("Segoe UI", 10))
            entry.grid(row=row, column=1, sticky=tk.EW, pady=8, padx=10)
            entry.insert(0, str(default_val))
            
            setattr(self, var_name, entry)
            
            if tooltip:
                tip = tk.Label(form_frame, text=tooltip, fg="#71717a", bg="#121214", font=("Segoe UI", 8))
                tip.grid(row=row, column=2, sticky=tk.W, pady=8)

        # Fields definition
        add_field(0, "Target Velocity (km/h):", "speed_var", 60.0, "(Vehicle cruise speed)")
        add_field(1, "Lane Change Start Time (s):", "t_start_var", 2.0, "(Starts path maneuver)")
        add_field(2, "Lane Change Duration (s):", "t_dur_var", 5.0, "(Time to transition)")
        add_field(3, "Simulation Stop Time (s):", "t_end_var", 13.0, "(Maneuver stop window)")
        add_field(4, "Lane Change Width (m):", "width_var", 5.0, "(Lateral offset amplitude)")
        add_field(5, "Start Length Margin (m):", "start_margin_var", 20.0, "(Extra terrain behind vehicle)")
        add_field(6, "End Length Margin (m):", "end_margin_var", 50.0, "(Extra terrain ahead at end)")
        add_field(7, "Mesh Grid Resolution (m):", "mesh_res_var", 0.25, "(du and dv spacing)")
        add_field(8, "Road Width (m):", "r_width_var", 8.0, "(Flat boundary width)")

        # ISO Class dropdown
        lbl = ttk.Label(form_frame, text="ISO 8608 Roughness Class:")
        lbl.grid(row=9, column=0, sticky=tk.W, pady=8)
        
        self.iso_box = ttk.Combobox(form_frame, values=[
            "A (Very Good - Highways / Motorways)",
            "B (Good - Main Roads / Old Highways)",
            "C (Average - Secondary Roads / Local Asphalt)",
            "D (Poor - Unpaved Roads / Cobblestone)",
            "E (Very Poor - Rough Dirt Roads / Damaged Streets)"
        ], state="readonly", font=("Segoe UI", 10))
        self.iso_box.set("C (Average - Secondary Roads / Local Asphalt)")
        self.iso_box.grid(row=9, column=1, sticky=tk.EW, pady=8, padx=10)
        
        tip = tk.Label(form_frame, text="(ISO 8608 standard classes)", fg="#71717a", bg="#121214", font=("Segoe UI", 8))
        tip.grid(row=9, column=2, sticky=tk.W, pady=8)

        # Checkbox for OBJ
        lbl = ttk.Label(form_frame, text="Generate 3D OBJ Mesh:")
        lbl.grid(row=10, column=0, sticky=tk.W, pady=8)
        
        self.obj_val = tk.BooleanVar(value=True)
        self.obj_check = ttk.Checkbutton(form_frame, variable=self.obj_val, style="TCheckbutton")
        self.obj_check.grid(row=10, column=1, sticky=tk.W, pady=8, padx=10)

        # Grid config
        form_frame.columnconfigure(1, weight=1)

        # Generate Button
        self.gen_btn = ttk.Button(self.root, text="GENERATE ROAD & PATH CENTERLINE", style="Accent.TButton", command=self.start_generation)
        self.gen_btn.pack(fill=tk.X, padx=20, pady=10)

        # Log Output Frame
        log_frame = tk.Frame(self.root, bg="#1e1e24", bd=1, relief=tk.SOLID)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

        log_lbl = tk.Label(log_frame, text="Execution Logs", bg="#1e1e24", fg="#a1a1aa", font=("Segoe UI", 9, "bold"))
        log_lbl.pack(anchor=tk.W, padx=10, pady=5)

        self.log_text = tk.Text(log_frame, bg="#0f0f11", fg="#a7f3d0", font=("Consolas", 9), wrap=tk.WORD, borderwidth=0)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # Redirect standard output
        sys.stdout = RedirectText(self.log_text)
        sys.stderr = RedirectText(self.log_text)

    def start_generation(self):
        # Disable button during run
        self.gen_btn.config(state=tk.DISABLED)
        self.log_text.delete(1.0, tk.END)
        
        # Run inside a separate thread so GUI doesn't freeze
        t = threading.Thread(target=self.run_generator)
        t.daemon = True
        t.start()

    def run_generator(self):
        try:
            # Parse parameters
            speed = float(self.speed_var.get())
            t_start = float(self.t_start_var.get())
            t_dur = float(self.t_dur_var.get())
            t_end = float(self.t_end_var.get())
            width = float(self.width_var.get())
            start_margin = float(self.start_margin_var.get())
            end_margin = float(self.end_margin_var.get())
            mesh_res = float(self.mesh_res_var.get())
            r_width = float(self.r_width_var.get())
            iso_class = self.iso_box.get()[0]
            gen_obj = self.obj_val.get()

            # Execute generation
            print("--- Executing Road Profile Generation ---")
            generate_road_profile(
                target_speed_kph=speed,
                t_start=t_start,
                t_duration=t_dur,
                t_end=t_end,
                width=width,
                start_length_margin=start_margin,
                end_length_margin=end_margin,
                mesh_resolution=mesh_res,
                v_width=r_width,
                iso_class=iso_class,
                generate_obj=gen_obj,
                base_dir=SCRIPT_DIR
            )
            print("\nSUCCESS: Path and terrain profiles updated in build/generated/ successfully!")
            messagebox.showinfo("Success", "Road profile generated successfully! Run 'build_fmus.bat' or compile via CMake to stage files.")
        except Exception as e:
            print(f"\nERROR: Generation failed. {e}")
            messagebox.showerror("Generation Failed", f"An error occurred: {e}")
        finally:
            self.gen_btn.config(state=tk.NORMAL)

if __name__ == "__main__":
    root = tk.Tk()
    app = RoadGeneratorGUI(root)
    root.mainloop()
