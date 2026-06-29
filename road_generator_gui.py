import os
import sys
import json
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import numpy as np

# Add src/road_generator to system path to import the generator
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROAD_GEN_DIR = os.path.join(SCRIPT_DIR, "src", "road_generator")
sys.path.append(ROAD_GEN_DIR)

try:
    from road_generator import (
        generate_road_profile,
        get_trajectory_pose,
        evaluate_profile,
        get_station_at_time,
        get_speed_at_time
    )
except ImportError:
    messagebox.showerror("Import Error", "Could not import road generator components. Make sure you are in the chrono_fmus root folder.")
    sys.exit(1)

SETTINGS_FILE = os.path.join(ROAD_GEN_DIR, "road_generator_settings.json")

SURFACE_PRESETS = {
    "Dry Concrete (Excellent)": {"iso": "A", "mu": 0.90},
    "Dry Asphalt (Good)": {"iso": "B", "mu": 0.85},
    "Wet Asphalt (Average)": {"iso": "C", "mu": 0.55},
    "Rainy / Poor Asphalt (Poor)": {"iso": "D", "mu": 0.45},
    "Snow (Average)": {"iso": "C", "mu": 0.25},
    "Ice (Average)": {"iso": "C", "mu": 0.10},
    "Cobblestone (Rough)": {"iso": "D", "mu": 0.60},
    "Offroad Dirt Road (Very Poor)": {"iso": "E", "mu": 0.45},
    "Rugged Offroad (Extremely Rough)": {"iso": "G", "mu": 0.35},
    "Manual Override": None
}

class RedirectText:
    """Helper to redirect stdout/stderr to a thread-safe Queue."""
    def __init__(self, log_queue):
        self.log_queue = log_queue

    def write(self, string):
        self.log_queue.put(string)

    def flush(self):
        pass

# Safe conversion utility to prevent conversion exceptions
def safe_float(value, fallback=0.0):
    try:
        # Strip off any non-numeric/prefix chars that might end up in boxes
        clean = "".join(c for c in str(value) if c.isdigit() or c in ['.', '-', '+', 'e', 'E'])
        return float(clean)
    except Exception:
        return fallback

# Helper mathematical functions for GUI real-time preview (using module imports)
def get_preview_points(speed_init, speed_target, t_speed_start, t_speed_dur, t_end, maneuver, t_start, t_dur, width, dwell, radius, direction, lead_in, start_margin, radius2=20.0, slalom_period=30.0, slalom_amplitude=2.0, theta1=45.0, theta2=180.0, clothoid_a=0.0005, vehicle_width=2.0):
    v_init = max(speed_init, 1.0) / 3.6
    v_target = max(speed_target, 1.0) / 3.6
    
    t_end_safe = max(t_end, 1.0)
    dt = t_end_safe / 100.0
    times = [i * dt for i in range(101)]
    points = []
    
    # Precompute clothoid array if needed
    s_clothoid = None
    x_clothoid = None
    y_clothoid = None
    if maneuver == "Constant Speed Spiral":
        s_clothoid = np.linspace(0.0, 2000.0, 20000)
        phi_clothoid = 0.5 * clothoid_a * s_clothoid**2
        x_clothoid = np.zeros(len(s_clothoid))
        y_clothoid = np.zeros(len(s_clothoid))
        ds_step = s_clothoid[1] - s_clothoid[0]
        x_clothoid[1:] = np.cumsum(np.cos(phi_clothoid[:-1])) * ds_step
        y_clothoid[1:] = np.cumsum(np.sin(phi_clothoid[:-1])) * ds_step

    # Package parameters for get_trajectory_pose
    params = {
        "v_init": v_init,
        "v_target": v_target,
        "t_speed_start": t_speed_start,
        "t_speed_dur": t_speed_dur,
        "start_length_margin": start_margin,
        "maneuver_type": maneuver,
        "t_start": t_start,
        "t_duration": t_dur,
        "width": width,
        "dwell_time": dwell,
        "radius": radius,
        "direction": direction,
        "lead_in_length": lead_in,
        "radius2": radius2,
        "slalom_period": slalom_period,
        "slalom_amplitude": slalom_amplitude,
        "theta1": theta1,
        "theta2": theta2,
        "clothoid_a": clothoid_a,
        "vehicle_width": vehicle_width,
        "s_clothoid": s_clothoid,
        "x_clothoid": x_clothoid,
        "y_clothoid": y_clothoid
    }

    for t in times:
        x, y, phi = get_trajectory_pose(t, params)
        points.append((x, y))
        
    return points

def get_slope_profile(u_length, slope_type, slope_val, slope_start, slope_dur):
    du = u_length / 100.0
    u_vals = [i * du for i in range(101)]
    z_vals = [0.0]
    
    for i in range(100):
        s_current = evaluate_profile(u_vals[i], slope_type, slope_val, slope_start, slope_dur)
        z_vals.append(z_vals[-1] + du * s_current)
        
    return list(zip(u_vals, z_vals))

class RoadGeneratorGUI:
    # Maneuver Defaults Dictionary for GUI reset
    MANEUVER_DEFAULTS = {
        "Straight Line": {},
        "Single Lane Change": {"t_start": 2.0, "t_duration": 5.0, "width": 5.0, "t_end": 13.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "Double Lane Change": {"t_start": 2.0, "t_duration": 5.0, "width": 5.0, "dwell_time": 2.0, "t_end": 15.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "Circular Path": {"radius": 50.0, "direction": "Left", "lead_in_length": 20.0, "t_end": 25.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "J-Turn": {"radius": 50.0, "direction": "Left", "lead_in_length": 20.0, "t_end": 15.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "U-Turn": {"radius": 50.0, "direction": "Left", "lead_in_length": 20.0, "t_end": 20.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "Slalom": {"lead_in_length": 20.0, "slalom_period": 30.0, "slalom_amplitude": 2.0, "t_end": 20.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "Sine with Dwell": {"t_start": 2.0, "t_duration": 5.0, "width": 5.0, "dwell_time": 2.0, "t_end": 15.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "Fishhook": {"radius": 50.0, "radius2": 20.0, "direction": "Left", "lead_in_length": 20.0, "theta1": 45.0, "theta2": 180.0, "t_end": 15.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "ISO 3888-2 Obstacle Avoidance": {"lead_in_length": 20.0, "vehicle_width": 2.0, "t_end": 15.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "Constant Speed Spiral": {"direction": "Left", "lead_in_length": 20.0, "clothoid_a": 0.0005, "t_end": 15.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "Braking in a Turn": {"radius": 50.0, "direction": "Left", "lead_in_length": 20.0, "speed_profile_initial_speed": 80.0, "speed_profile_target_speed": 0.0, "speed_profile_time_start": 3.0, "speed_profile_duration": 2.0, "t_end": 8.0}
    }

    def __init__(self, root):
        self.root = root
        self.root.title("Chrono Test Case Control Center & Road Designer")
        self.root.geometry("1024x760")
        self.root.configure(bg="#121214")
        self.root.resizable(True, True)

        self.log_queue = queue.Queue()

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
        self.style.configure("TLabel", background="#121214", foreground="#e4e4e7", font=("Segoe UI", 9))
        self.style.configure("TEntry", fieldbackground="#1e1e24", foreground="#ffffff", insertcolor="#ffffff", bordercolor="#3f3f46")
        self.style.configure("TCombobox", fieldbackground="#1e1e24", foreground="#ffffff", selectbackground="#4f46e5")
        self.style.configure("TLabelframe", background="#121214", foreground="#6366f1", bordercolor="#3f3f46", font=("Segoe UI", 10, "bold"))
        self.style.configure("TLabelframe.Label", background="#121214", foreground="#6366f1")
        self.style.configure("TCheckbutton", background="#121214", foreground="#e4e4e7", font=("Segoe UI", 9))
        
        # Notebook (Tab) Styling
        self.style.configure("TNotebook", background="#121214", borderwidth=0)
        self.style.configure("TNotebook.Tab", background="#1e1e24", foreground="#a1a1aa", font=("Segoe UI", 9, "bold"), padding=[10, 4])
        self.style.map("TNotebook.Tab",
                       background=[("selected", "#4f46e5"), ("active", "#27272a")],
                       foreground=[("selected", "#ffffff"), ("active", "#ffffff")])

        self.style.map("TEntry",
                       foreground=[("active", "#ffffff"), ("disabled", "#52525b")],
                       fieldbackground=[("active", "#1e1e24"), ("disabled", "#18181b")])
        self.style.map("TCombobox",
                       foreground=[("active", "#ffffff"), ("disabled", "#52525b")],
                       fieldbackground=[("active", "#1e1e24"), ("disabled", "#18181b")])
        
        # Primary Accent Button
        self.style.configure("Accent.TButton", 
                              background="#4f46e5", 
                              foreground="#ffffff", 
                              font=("Segoe UI", 11, "bold"), 
                              borderwidth=0, 
                              focuscolor="none")
        self.style.map("Accent.TButton", 
                       background=[("active", "#6366f1"), ("pressed", "#4338ca")])

        self.loaded_params = self.load_settings()
        self.canvas = None
        self.create_widgets()
        self.apply_preset()
        self.update_maneuver_visibility()
        self.update_steering_visibility()
        self.update_slope_visibility()
        self.update_banking_visibility()
        self.bind_all_fields()

        # Bind window resize event
        self.root.bind("<Configure>", self.on_window_resize)
        self.update_preview()

        # Start queue log poller
        self.root.after(100, self.process_log_queue)

    def load_settings(self):
        if os.path.exists(SETTINGS_FILE):
            try:
                with open(SETTINGS_FILE, "r") as f:
                    return json.load(f)
            except Exception as e:
                print(f"Error loading settings: {e}")
        return {}

    def save_settings(self):
        try:
            params = {
                "target_speed_kph": safe_float(self.speed_target_var.get(), 60.0),
                "t_start": safe_float(self.t_start_var.get(), 2.0),
                "t_duration": safe_float(self.t_dur_var.get(), 5.0),
                "t_end": safe_float(self.t_end_var.get(), 13.0),
                "width": safe_float(self.width_var.get(), 5.0),
                "start_length_margin": safe_float(self.start_margin_var.get(), 20.0),
                "end_length_margin": safe_float(self.end_margin_var.get(), 50.0),
                "mesh_resolution": safe_float(self.mesh_res_var.get(), 0.06),
                "v_width": safe_float(self.r_width_var.get(), 8.0),
                "iso_class": self.iso_box.get().split(" ")[0],
                "generate_obj": self.obj_val.get(),
                
                "maneuver_type": self.maneuver_box.get(),
                "dwell_time": safe_float(self.dwell_var.get(), 2.0),
                "radius": safe_float(self.radius_var.get(), 50.0),
                "direction": self.direction_box.get(),
                "lead_in_length": safe_float(self.lead_in_var.get(), 20.0),

                # Extended parameters
                "radius2": safe_float(self.radius2_var.get(), 20.0),
                "slalom_period": safe_float(self.slalom_period_var.get(), 30.0),
                "slalom_amplitude": safe_float(self.slalom_amp_var.get(), 2.0),
                "theta1": safe_float(self.theta1_var.get(), 45.0),
                "theta2": safe_float(self.theta2_var.get(), 180.0),
                "clothoid_a": safe_float(self.clothoid_a_var.get(), 0.0005),
                "vehicle_width": safe_float(self.vehicle_width_var.get(), 2.0),
                
                "slope_type": self.slope_type_box.get(),
                "slope_value": safe_float(self.slope_val_var.get(), 0.0),
                "slope_start": safe_float(self.slope_start_var.get(), 30.0),
                "slope_duration": safe_float(self.slope_dur_var.get(), 20.0),
                
                "banking_type": self.banking_type_box.get(),
                "banking_value": safe_float(self.banking_val_var.get(), 0.0),
                "banking_start": safe_float(self.banking_start_var.get(), 30.0),
                "banking_duration": safe_float(self.banking_dur_var.get(), 20.0),
                
                "mu_value": safe_float(self.mu_var.get(), 0.85),

                # Speed Profile parameters
                "speed_profile_time_start": safe_float(self.speed_start_var.get(), 2.0),
                "speed_profile_duration": safe_float(self.speed_dur_var.get(), 5.0),
                "speed_profile_initial_speed": safe_float(self.speed_init_var.get(), 60.0),
                "speed_profile_target_speed": safe_float(self.speed_target_var.get(), 60.0),
                
                "steering_type": 1 if self.steer_type_box.get() == "Stanley" else 0,
                "look_ahead_dist": safe_float(self.look_ahead_var.get(), 3.615358),
                "Kp_steering": safe_float(self.kp_steer_var.get(), 2.398832),
                "Ki_steering": safe_float(self.ki_steer_var.get(), 0.0),
                "Kd_steering": safe_float(self.kd_steer_var.get(), 0.0),
                "stanley_dead_zone": safe_float(self.dead_zone_var.get(), 0.010965),
                "max_wheel_turn_angle": safe_float(self.max_turn_var.get(), 25.0),
                
                "Kp_speed": safe_float(self.kp_speed_var.get(), 0.868900),
                "Ki_speed": safe_float(self.ki_speed_var.get(), 0.436516),
                "Kd_speed": safe_float(self.kd_speed_var.get(), 0.0),

                # Roughness Discretization parameters
                "roughness_Nf": int(safe_float(self.roughness_nf_var.get(), 512)),
                "roughness_Ntheta": int(safe_float(self.roughness_nt_var.get(), 32))
            }
            with open(SETTINGS_FILE, "w") as f:
                json.dump(params, f, indent=4)
        except Exception as e:
            print(f"Error saving settings: {e}")

    def update_entry(self, entry, value, state=tk.NORMAL):
        """Safely updates a Tkinter Entry value regardless of its state."""
        entry.config(state=tk.NORMAL)
        entry.delete(0, tk.END)
        entry.insert(0, str(value))
        entry.config(state=state)

    def add_grid_entry(self, parent, row, col, label_text, default_val):
        lbl = ttk.Label(parent, text=label_text, font=("Segoe UI", 8))
        lbl.grid(row=row, column=col, sticky=tk.W, pady=2, padx=4)
        entry = ttk.Entry(parent, font=("Segoe UI", 8), width=10)
        entry.grid(row=row, column=col+1, sticky=tk.EW, pady=2, padx=4)
        entry.insert(0, str(default_val))
        return entry

    def add_grid_combobox(self, parent, row, col, label_text, values, default_val, callback=None):
        lbl = ttk.Label(parent, text=label_text, font=("Segoe UI", 8))
        lbl.grid(row=row, column=col, sticky=tk.W, pady=2, padx=4)
        box = ttk.Combobox(parent, values=values, state="readonly", font=("Segoe UI", 8), width=10)
        box.set(default_val)
        box.grid(row=row, column=col+1, sticky=tk.EW, pady=2, padx=4)
        if callback:
            box.bind("<<ComboboxSelected>>", callback)
        return box

    def create_widgets(self):
        # Header Label
        header_frame = tk.Frame(self.root, bg="#1a1a1e", height=70)
        header_frame.pack(fill=tk.X)
        header_frame.pack_propagate(False)

        header_lbl = tk.Label(header_frame, 
                               text="TEST CASE CONTROL CENTER & ROAD DESIGNER", 
                               fg="#6366f1", 
                               bg="#1a1a1e", 
                               font=("Segoe UI", 12, "bold"))
        header_lbl.pack(pady=8)
        
        sub_lbl = tk.Label(header_frame, 
                           text="Chrono Vehicle FMI Co-Simulation Suite", 
                           fg="#a1a1aa", 
                           bg="#1a1a1e", 
                           font=("Segoe UI", 8, "italic"))
        sub_lbl.pack()

        # Split layout: Left panel (inputs), Right panel (preview / logs)
        main_layout = tk.Frame(self.root, bg="#121214")
        main_layout.pack(fill=tk.BOTH, expand=True)

        left_panel = tk.Frame(main_layout, bg="#121214", width=450)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, padx=15, pady=10)
        left_panel.pack_propagate(False)

        right_panel = tk.Frame(main_layout, bg="#121214")
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=15, pady=10)

        # Tabbed interface to compress left panel vertical space
        self.notebook = ttk.Notebook(left_panel)
        self.notebook.pack(fill=tk.BOTH, expand=True, pady=5)

        tab_maneuver = tk.Frame(self.notebook, bg="#121214")
        tab_road = tk.Frame(self.notebook, bg="#121214")
        tab_controller = tk.Frame(self.notebook, bg="#121214")

        self.notebook.add(tab_maneuver, text=" Scenario & Maneuver ")
        self.notebook.add(tab_road, text=" Road & Environment ")
        self.notebook.add(tab_controller, text=" Controller Gains ")

        # Build Tab 1
        self.build_maneuver_tab(tab_maneuver)

        # Build Tab 2
        self.build_road_tab(tab_road)

        # Build Tab 3
        self.build_controller_tab(tab_controller)

        # Build Right Panel (Canvas + Log Output)
        self.build_right_panel(right_panel)

    def build_maneuver_tab(self, parent):
        # Card 1: Scenario Configuration
        scen_frame = ttk.LabelFrame(parent, text=" 1. Scenario & Speed Configuration ")
        scen_frame.pack(fill=tk.X, pady=5)
        
        self.maneuver_box = self.add_combobox_field(scen_frame, 0, "Maneuver Type:", 
            ["Straight Line", "Single Lane Change", "Double Lane Change", "Circular Path", "J-Turn", "U-Turn", "Slalom", "Sine with Dwell", "Fishhook", "ISO 3888-2 Obstacle Avoidance", "Constant Speed Spiral", "Braking in a Turn"],
            self.loaded_params.get("maneuver_type", "Single Lane Change"), self.update_maneuver_visibility)
        
        self.r_width_var = self.add_entry_field(scen_frame, 1, "Road Width (m):", self.loaded_params.get("v_width", 8.0))
        self.t_end_var = self.add_entry_field(scen_frame, 2, "Simulation Stop Time (s):", self.loaded_params.get("t_end", 13.0))
        
        self.speed_init_var = self.add_entry_field(scen_frame, 3, "Initial Speed (km/h):", self.loaded_params.get("speed_profile_initial_speed", 60.0))
        self.speed_target_var = self.add_entry_field(scen_frame, 4, "Target Speed (km/h):", self.loaded_params.get("speed_profile_target_speed", 60.0))
        self.speed_start_var = self.add_entry_field(scen_frame, 5, "Speed Ramp Start Time (s):", self.loaded_params.get("speed_profile_time_start", 2.0))
        self.speed_dur_var = self.add_entry_field(scen_frame, 6, "Speed Ramp Duration (s):", self.loaded_params.get("speed_profile_duration", 5.0))

        # Card 2: Maneuver Details (Static grid layout to display all parameters cleanly)
        self.man_frame = ttk.LabelFrame(parent, text=" 2. Maneuver Parameters ")
        self.man_frame.pack(fill=tk.X, pady=5)

        self.man_frame.columnconfigure(0, weight=1)
        self.man_frame.columnconfigure(1, weight=2)
        self.man_frame.columnconfigure(2, weight=1)
        self.man_frame.columnconfigure(3, weight=2)

        # Row 0: Margins
        self.start_margin_var = self.add_grid_entry(self.man_frame, 0, 0, "Start Margin (m):", self.loaded_params.get("start_length_margin", 20.0))
        self.end_margin_var = self.add_grid_entry(self.man_frame, 0, 2, "End Margin (m):", self.loaded_params.get("end_length_margin", 50.0))

        # Row 1: Start Time, Transition Dur
        self.t_start_var = self.add_grid_entry(self.man_frame, 1, 0, "Maneuver Start (s):", self.loaded_params.get("t_start", 2.0))
        self.t_dur_var = self.add_grid_entry(self.man_frame, 1, 2, "Transition (s):", self.loaded_params.get("t_duration", 5.0))

        # Row 2: Lateral Offset, Dwell Time
        self.width_var = self.add_grid_entry(self.man_frame, 2, 0, "Lateral Offset (m):", self.loaded_params.get("width", 5.0))
        self.dwell_var = self.add_grid_entry(self.man_frame, 2, 2, "Dwell Time (s):", self.loaded_params.get("dwell_time", 2.0))

        # Row 3: Circular Radius, Turn Direction
        self.radius_var = self.add_grid_entry(self.man_frame, 3, 0, "Circle Radius (m):", self.loaded_params.get("radius", 50.0))
        self.direction_box = self.add_grid_combobox(self.man_frame, 3, 2, "Direction:", ["Left", "Right"], self.loaded_params.get("direction", "Left"))

        # Row 4: Lead-in Length, Vehicle Width
        self.lead_in_var = self.add_grid_entry(self.man_frame, 4, 0, "Lead-in Len (m):", self.loaded_params.get("lead_in_length", 20.0))
        self.vehicle_width_var = self.add_grid_entry(self.man_frame, 4, 2, "Vehicle Width (m):", self.loaded_params.get("vehicle_width", 2.0))

        # Row 5: Slalom Period, Slalom Amp
        self.slalom_period_var = self.add_grid_entry(self.man_frame, 5, 0, "Slalom Period (m):", self.loaded_params.get("slalom_period", 30.0))
        self.slalom_amp_var = self.add_grid_entry(self.man_frame, 5, 2, "Slalom Amp (m):", self.loaded_params.get("slalom_amplitude", 2.0))

        # Row 6: Fishhook Radius 2, Clothoid Param A
        self.radius2_var = self.add_grid_entry(self.man_frame, 6, 0, "Fishhook Rad 2 (m):", self.loaded_params.get("radius2", 20.0))
        self.clothoid_a_var = self.add_grid_entry(self.man_frame, 6, 2, "Clothoid Param A:", self.loaded_params.get("clothoid_a", 0.0005))

        # Row 7: Fishhook Angle 1, Fishhook Angle 2
        self.theta1_var = self.add_grid_entry(self.man_frame, 7, 0, "Turn 1 Angle (deg):", self.loaded_params.get("theta1", 45.0))
        self.theta2_var = self.add_grid_entry(self.man_frame, 7, 2, "Turn 2 Angle (deg):", self.loaded_params.get("theta2", 180.0))

        # Row 8: Reset Button
        self.reset_btn = ttk.Button(self.man_frame, text="Reset to Maneuver Defaults", command=self.reset_maneuver_defaults)
        self.reset_btn.grid(row=8, column=0, columnspan=4, pady=8, padx=10, sticky=tk.EW)

    def build_road_tab(self, parent):
        # Card 1: Road Surface presets
        road_frame = ttk.LabelFrame(parent, text=" 1. Road Surface & Friction ")
        road_frame.pack(fill=tk.X, pady=5)
        
        preset_values = list(SURFACE_PRESETS.keys())
        default_preset = "Dry Asphalt (Good)"
        loaded_iso = self.loaded_params.get("iso_class", "C")
        loaded_mu = self.loaded_params.get("mu_value", 0.85)
        for k, v in SURFACE_PRESETS.items():
            if v and v["iso"] == loaded_iso and abs(v["mu"] - loaded_mu) < 0.01:
                default_preset = k
                break
        
        self.preset_box = self.add_combobox_field(road_frame, 0, "Surface Preset:", preset_values, default_preset, self.apply_preset)
        
        iso_options = [
            "A (Very Good - Highways / Motorways)",
            "B (Good - Main Roads / Old Highways)",
            "C (Average - Secondary Roads / Local Asphalt)",
            "D (Poor - Unpaved Roads / Cobblestone)",
            "E (Very Poor - Rough Dirt Roads)",
            "F (Damaged Dirt Streets)",
            "G (Rugged Terrain)",
            "H (Severely Damaged / Rugged Offroad)"
        ]
        default_iso_str = [x for x in iso_options if x.startswith(loaded_iso)][0]
        self.iso_box = self.add_combobox_field(road_frame, 1, "ISO Roughness Class:", iso_options, default_iso_str, self.update_preset_custom)
        self.mu_var = self.add_entry_field(road_frame, 2, "Manual friction coefficient (mu):", loaded_mu)
        self.mu_var.bind("<KeyRelease>", self.update_preset_custom)

        # Card 2: Slope & Banking
        sb_frame = ttk.LabelFrame(parent, text=" 2. Road Slope & Banking ")
        sb_frame.pack(fill=tk.X, pady=5)
        
        self.slope_type_box = self.add_combobox_field(sb_frame, 0, "Slope Profile Type:", ["None", "Constant", "Smooth Step"], self.loaded_params.get("slope_type", "None"), self.update_slope_visibility)
        self.slope_val_var = self.add_entry_field(sb_frame, 1, "Slope Value (m/m):", self.loaded_params.get("slope_value", 0.02))
        self.slope_start_var = self.add_entry_field(sb_frame, 2, "Slope Start Station (m):", self.loaded_params.get("slope_start", 30.0))
        self.slope_dur_var = self.add_entry_field(sb_frame, 3, "Slope Transition Dist (m):", self.loaded_params.get("slope_duration", 20.0))
        
        self.banking_type_box = self.add_combobox_field(sb_frame, 4, "Banking Profile Type:", ["None", "Constant", "Smooth Step", "Link to Curvature"], self.loaded_params.get("banking_type", "None"), self.update_banking_visibility)
        self.banking_val_var = self.add_entry_field(sb_frame, 5, "Banking Value (m/m):", self.loaded_params.get("banking_value", 0.04))
        self.banking_start_var = self.add_entry_field(sb_frame, 6, "Banking Start Station (m):", self.loaded_params.get("banking_start", 30.0))
        self.banking_dur_var = self.add_entry_field(sb_frame, 7, "Banking Transition Dist (m):", self.loaded_params.get("banking_duration", 20.0))

    def build_controller_tab(self, parent):
        # Card 1: Controller & Driver Configuration
        ctrl_frame = ttk.LabelFrame(parent, text=" Steering & Cruise Controller Config ")
        ctrl_frame.pack(fill=tk.X, pady=5)
        
        saved_steering = "Stanley" if self.loaded_params.get("steering_type", 1) == 1 else "PID"
        self.steer_type_box = self.add_combobox_field(ctrl_frame, 0, "Steering Type:", ["Stanley", "PID"], saved_steering, self.update_steering_visibility)
        self.look_ahead_var = self.add_entry_field(ctrl_frame, 1, "Look-Ahead Distance (m):", self.loaded_params.get("look_ahead_dist", 3.615358))
        
        # Steering gains
        gains_frame = tk.Frame(ctrl_frame, bg="#121214")
        gains_frame.grid(row=2, column=0, columnspan=3, sticky=tk.EW, pady=4, padx=10)
        
        tk.Label(gains_frame, text="Steer Kp:", bg="#121214", fg="#a1a1aa", font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=2)
        self.kp_steer_var = ttk.Entry(gains_frame, width=8, font=("Segoe UI", 9))
        self.kp_steer_var.insert(0, str(self.loaded_params.get("Kp_steering", 2.398832)))
        self.kp_steer_var.pack(side=tk.LEFT, padx=4)
        
        tk.Label(gains_frame, text="Ki:", bg="#121214", fg="#a1a1aa", font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=2)
        self.ki_steer_var = ttk.Entry(gains_frame, width=8, font=("Segoe UI", 9))
        self.ki_steer_var.insert(0, str(self.loaded_params.get("Ki_steering", 0.0)))
        self.ki_steer_var.pack(side=tk.LEFT, padx=4)

        tk.Label(gains_frame, text="Kd:", bg="#121214", fg="#a1a1aa", font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=2)
        self.kd_steer_var = ttk.Entry(gains_frame, width=8, font=("Segoe UI", 9))
        self.kd_steer_var.insert(0, str(self.loaded_params.get("Kd_steering", 0.0)))
        self.kd_steer_var.pack(side=tk.LEFT, padx=4)

        self.dead_zone_var = self.add_entry_field(ctrl_frame, 3, "Stanley Dead Zone (m):", self.loaded_params.get("stanley_dead_zone", 0.010965))
        self.max_turn_var = self.add_entry_field(ctrl_frame, 4, "Max Wheel Turn (deg):", self.loaded_params.get("max_wheel_turn_angle", 25.0))

        # Cruise speed gains
        speed_gains_frame = tk.Frame(ctrl_frame, bg="#121214")
        speed_gains_frame.grid(row=5, column=0, columnspan=3, sticky=tk.EW, pady=4, padx=10)
        
        tk.Label(speed_gains_frame, text="Speed Kp:", bg="#121214", fg="#a1a1aa", font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=2)
        self.kp_speed_var = ttk.Entry(speed_gains_frame, width=8, font=("Segoe UI", 9))
        self.kp_speed_var.insert(0, str(self.loaded_params.get("Kp_speed", 0.868900)))
        self.kp_speed_var.pack(side=tk.LEFT, padx=4)
        
        tk.Label(speed_gains_frame, text="Ki:", bg="#121214", fg="#a1a1aa", font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=2)
        self.ki_speed_var = ttk.Entry(speed_gains_frame, width=8, font=("Segoe UI", 9))
        self.ki_speed_var.insert(0, str(self.loaded_params.get("Ki_speed", 0.436516)))
        self.ki_speed_var.pack(side=tk.LEFT, padx=4)

        tk.Label(speed_gains_frame, text="Kd:", bg="#121214", fg="#a1a1aa", font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=2)
        self.kd_speed_var = ttk.Entry(speed_gains_frame, width=8, font=("Segoe UI", 9))
        self.kd_speed_var.insert(0, str(self.loaded_params.get("Kd_speed", 0.0)))
        self.kd_speed_var.pack(side=tk.LEFT, padx=4)

        # Resolution card
        res_frame = ttk.LabelFrame(parent, text=" Export Resolution & Formats ")
        res_frame.pack(fill=tk.X, pady=5)
        
        lbl_res = ttk.Label(res_frame, text="Mesh Resolution (m):")
        lbl_res.grid(row=0, column=0, sticky=tk.W, pady=4, padx=10)
        self.mesh_res_var = ttk.Entry(res_frame, width=15, font=("Segoe UI", 9))
        self.mesh_res_var.grid(row=0, column=1, sticky=tk.W, pady=4, padx=10)
        self.mesh_res_var.insert(0, str(self.loaded_params.get("mesh_resolution", 0.06)))

        # Default generate_obj to False
        self.obj_val = tk.BooleanVar(value=self.loaded_params.get("generate_obj", False))
        self.obj_check = ttk.Checkbutton(res_frame, text="Generate 3D OBJ Mesh File", variable=self.obj_val, style="TCheckbutton")
        self.obj_check.grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=4, padx=10)

        # Nf and Ntheta Roughness Discretization parameters
        lbl_nf = ttk.Label(res_frame, text="Roughness Freq Bands (Nf):")
        lbl_nf.grid(row=2, column=0, sticky=tk.W, pady=4, padx=10)
        self.roughness_nf_var = ttk.Entry(res_frame, width=15, font=("Segoe UI", 9))
        self.roughness_nf_var.grid(row=2, column=1, sticky=tk.W, pady=4, padx=10)
        self.roughness_nf_var.insert(0, str(self.loaded_params.get("roughness_Nf", 512)))

        lbl_nt = ttk.Label(res_frame, text="Roughness Angle Divs (Ntheta):")
        lbl_nt.grid(row=3, column=0, sticky=tk.W, pady=4, padx=10)
        self.roughness_nt_var = ttk.Entry(res_frame, width=15, font=("Segoe UI", 9))
        self.roughness_nt_var.grid(row=3, column=1, sticky=tk.W, pady=4, padx=10)
        self.roughness_nt_var.insert(0, str(self.loaded_params.get("roughness_Ntheta", 32)))

        # Bind fields inside controllers tab
        self.kp_steer_var.bind("<KeyRelease>", self.update_preview)
        self.ki_steer_var.bind("<KeyRelease>", self.update_preview)
        self.kd_steer_var.bind("<KeyRelease>", self.update_preview)
        self.kp_speed_var.bind("<KeyRelease>", self.update_preview)
        self.ki_speed_var.bind("<KeyRelease>", self.update_preview)
        self.kd_speed_var.bind("<KeyRelease>", self.update_preview)
        self.mesh_res_var.bind("<KeyRelease>", self.update_preview)
        self.roughness_nf_var.bind("<KeyRelease>", self.update_preview)
        self.roughness_nt_var.bind("<KeyRelease>", self.update_preview)

    def build_right_panel(self, parent):
        # 1. Preview Canvas (Increased height to 450px to fit Bank Angle plot)
        preview_lbl = tk.Label(parent, text="Real-Time Path & Profile Preview", bg="#121214", fg="#6366f1", font=("Segoe UI", 10, "bold"))
        preview_lbl.pack(anchor=tk.W, pady=2)
        
        self.canvas = tk.Canvas(parent, bg="#09090b", height=450, bd=1, relief=tk.SOLID, highlightthickness=0)
        self.canvas.pack(fill=tk.X, pady=5)

        # 2. Generate button
        self.gen_btn = ttk.Button(parent, text="GENERATE ROAD & TEST CASE CONFIG", style="Accent.TButton", command=self.start_generation)
        self.gen_btn.pack(fill=tk.X, pady=10)

        # 3. Execution log area
        log_lbl = tk.Label(parent, text="Execution Output Logs", bg="#121214", fg="#a1a1aa", font=("Segoe UI", 9, "bold"))
        log_lbl.pack(anchor=tk.W, pady=2)
        
        log_frame = tk.Frame(parent, bg="#1e1e24", bd=1, relief=tk.SOLID)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.log_text = tk.Text(log_frame, bg="#0f0f11", fg="#a7f3d0", font=("Consolas", 9), wrap=tk.WORD, borderwidth=0)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Redirect standard output
        sys.stdout = RedirectText(self.log_queue)
        sys.stderr = RedirectText(self.log_queue)

    def add_entry_field(self, parent, row, label_text, default_val):
        lbl = ttk.Label(parent, text=label_text)
        lbl.grid(row=row, column=0, sticky=tk.W, pady=1, padx=10)
        entry = ttk.Entry(parent, font=("Segoe UI", 9))
        entry.grid(row=row, column=1, sticky=tk.EW, pady=1, padx=10)
        entry.insert(0, str(default_val))
        parent.columnconfigure(1, weight=1)
        return entry

    def add_combobox_field(self, parent, row, label_text, values, default_val, callback=None):
        lbl = ttk.Label(parent, text=label_text)
        lbl.grid(row=row, column=0, sticky=tk.W, pady=1, padx=10)
        box = ttk.Combobox(parent, values=values, state="readonly", font=("Segoe UI", 9))
        box.set(default_val)
        box.grid(row=row, column=1, sticky=tk.EW, pady=1, padx=10)
        if callback:
            box.bind("<<ComboboxSelected>>", callback)
        parent.columnconfigure(1, weight=1)
        return box

    def bind_all_fields(self):
        for var_name in ["speed_init_var", "speed_target_var", "speed_start_var", "speed_dur_var", "t_end_var", 
                         "r_width_var", "t_start_var", "t_dur_var", "width_var", "dwell_var",
                         "radius_var", "lead_in_var", "start_margin_var", "end_margin_var", "slope_val_var", 
                         "slope_start_var", "slope_dur_var", "banking_val_var", "banking_start_var", "banking_dur_var",
                         "radius2_var", "slalom_period_var", "slalom_amp_var", "theta1_var", "theta2_var",
                         "clothoid_a_var", "vehicle_width_var", "roughness_nf_var", "roughness_nt_var"]:
            widget = getattr(self, var_name)
            widget.bind("<KeyRelease>", self.update_preview)

        self.direction_box.bind("<<ComboboxSelected>>", self.update_preview)

    def update_maneuver_visibility(self, event=None):
        maneuver = self.maneuver_box.get()
        
        # Default all variables to disabled first
        for var_name in ["t_start_var", "t_dur_var", "width_var", "dwell_var", "radius_var", 
                         "direction_box", "lead_in_var", "radius2_var", "slalom_period_var", 
                         "slalom_amp_var", "theta1_var", "theta2_var", "clothoid_a_var", 
                         "vehicle_width_var"]:
            getattr(self, var_name).config(state=tk.DISABLED)
            
        # Enable based on maneuver
        enabled_vars = []
        if maneuver == "Single Lane Change":
            enabled_vars = ["t_start_var", "t_dur_var", "width_var"]
        elif maneuver == "Double Lane Change":
            enabled_vars = ["t_start_var", "t_dur_var", "width_var", "dwell_var"]
        elif maneuver in ["Circular Path", "J-Turn", "U-Turn", "Braking in a Turn"]:
            enabled_vars = ["radius_var", "direction_box", "lead_in_var"]
        elif maneuver == "Slalom":
            enabled_vars = ["lead_in_var", "slalom_period_var", "slalom_amp_var"]
        elif maneuver == "Sine with Dwell":
            enabled_vars = ["t_start_var", "t_dur_var", "width_var", "dwell_var"]
        elif maneuver == "Fishhook":
            enabled_vars = ["radius_var", "direction_box", "lead_in_var", "radius2_var", "theta1_var", "theta2_var"]
        elif maneuver == "ISO 3888-2 Obstacle Avoidance":
            enabled_vars = ["lead_in_var", "vehicle_width_var"]
        elif maneuver == "Constant Speed Spiral":
            enabled_vars = ["direction_box", "lead_in_var", "clothoid_a_var"]
            
        # Set state to normal/readonly
        for var_name in enabled_vars:
            widget = getattr(self, var_name)
            if isinstance(widget, ttk.Combobox):
                widget.config(state="readonly")
            else:
                widget.config(state=tk.NORMAL)
                
        # Handle special braking or circular time calculations
        if maneuver == "Braking in a Turn":
            self.update_entry(self.speed_init_var, "80.0")
            self.update_entry(self.speed_target_var, "0.0")
            self.update_entry(self.speed_start_var, "3.0")
            self.update_entry(self.speed_dur_var, "2.0")
        elif maneuver == "Circular Path":
            try:
                rad = safe_float(self.radius_var.get(), 50.0)
                speed = safe_float(self.speed_target_var.get(), 60.0) / 3.6
                t_circle = 2.0 * np.pi * rad / speed
                t_stop = safe_float(self.start_margin_var.get(), 20.0) / speed + safe_float(self.lead_in_var.get(), 20.0) / speed + t_circle + 2.0
                self.update_entry(self.t_end_var, f"{t_stop:.1f}")
            except Exception:
                pass
                
        self.update_preview()

    def reset_maneuver_defaults(self):
        maneuver = self.maneuver_box.get()
        defaults = self.MANEUVER_DEFAULTS.get(maneuver, {})
        
        # Map of config keys to GUI entry/combobox variables
        mapping = {
            "speed_profile_initial_speed": self.speed_init_var,
            "speed_profile_target_speed": self.speed_target_var,
            "speed_profile_time_start": self.speed_start_var,
            "speed_profile_duration": self.speed_dur_var,
            "t_start": self.t_start_var,
            "t_duration": self.t_dur_var,
            "t_end": self.t_end_var,
            "width": self.width_var,
            "dwell_time": self.dwell_var,
            "radius": self.radius_var,
            "direction": self.direction_box,
            "lead_in_length": self.lead_in_var,
            "radius2": self.radius2_var,
            "slalom_period": self.slalom_period_var,
            "slalom_amplitude": self.slalom_amp_var,
            "theta1": self.theta1_var,
            "theta2": self.theta2_var,
            "clothoid_a": self.clothoid_a_var,
            "vehicle_width": self.vehicle_width_var
        }
        
        for key, default_val in defaults.items():
            widget = mapping.get(key)
            if widget:
                if isinstance(widget, ttk.Combobox):
                    widget.set(default_val)
                else:
                    self.update_entry(widget, default_val)
                    
        # Update visibility and preview
        self.update_maneuver_visibility()
        self.update_preview()

    def update_slope_visibility(self, event=None):
        st = self.slope_type_box.get()
        if st == "None":
            self.slope_val_var.config(state=tk.DISABLED)
            self.slope_start_var.config(state=tk.DISABLED)
            self.slope_dur_var.config(state=tk.DISABLED)
        elif st == "Constant":
            self.slope_val_var.config(state=tk.NORMAL)
            self.slope_start_var.config(state=tk.DISABLED)
            self.slope_dur_var.config(state=tk.DISABLED)
        elif st == "Smooth Step":
            self.slope_val_var.config(state=tk.NORMAL)
            self.slope_start_var.config(state=tk.NORMAL)
            self.slope_dur_var.config(state=tk.NORMAL)
        self.update_preview()

    def update_banking_visibility(self, event=None):
        bt = self.banking_type_box.get()
        if bt == "None":
            self.banking_val_var.config(state=tk.DISABLED)
            self.banking_start_var.config(state=tk.DISABLED)
            self.banking_dur_var.config(state=tk.DISABLED)
        elif bt in ["Constant", "Link to Curvature"]:
            self.banking_val_var.config(state=tk.NORMAL)
            self.banking_start_var.config(state=tk.DISABLED)
            self.banking_dur_var.config(state=tk.DISABLED)
        elif bt == "Smooth Step":
            self.banking_val_var.config(state=tk.NORMAL)
            self.banking_start_var.config(state=tk.NORMAL)
            self.banking_dur_var.config(state=tk.NORMAL)
        self.update_preview()

    def update_steering_visibility(self, event=None):
        st = self.steer_type_box.get()
        if st == "Stanley":
            self.dead_zone_var.config(state=tk.NORMAL)
            self.look_ahead_var.config(state=tk.NORMAL)
            self.kp_steer_var.config(state=tk.NORMAL)
            
            # Grey out PID-specific gains (Ki, Kd)
            self.ki_steer_var.config(state=tk.DISABLED)
            self.kd_steer_var.config(state=tk.DISABLED)
            
            # Prefill Stanley optimized defaults (from optimization report)
            self.update_entry(self.kp_steer_var, "2.398832")
            self.update_entry(self.ki_steer_var, "0.0", state=tk.DISABLED)
            self.update_entry(self.kd_steer_var, "0.0", state=tk.DISABLED)
            self.update_entry(self.look_ahead_var, "3.615358")
            self.update_entry(self.dead_zone_var, "0.010965")
        else:
            self.dead_zone_var.config(state=tk.DISABLED)
            self.look_ahead_var.config(state=tk.NORMAL)
            self.kp_steer_var.config(state=tk.NORMAL)
            self.ki_steer_var.config(state=tk.NORMAL)
            self.kd_steer_var.config(state=tk.NORMAL)
            
            # Prefill PID optimized defaults (from optimization report)
            self.update_entry(self.kp_steer_var, "1.047129")
            self.update_entry(self.ki_steer_var, "0.010000")
            self.update_entry(self.kd_steer_var, "0.0")
            self.update_entry(self.look_ahead_var, "4.990583")
            self.update_entry(self.dead_zone_var, "0.0", state=tk.DISABLED)

    def apply_preset(self, event=None):
        preset_name = self.preset_box.get()
        preset = SURFACE_PRESETS[preset_name]
        
        if preset:
            for val in self.iso_box['values']:
                if val.startswith(preset["iso"]):
                    self.iso_box.set(val)
                    break
            self.iso_box.config(state=tk.DISABLED)
            
            # Fix Entry disabled update bug (temporarily unlock state to write to it)
            self.update_entry(self.mu_var, str(preset["mu"]), state=tk.DISABLED)
        else:
            self.iso_box.config(state="readonly")
            self.mu_var.config(state=tk.NORMAL)
            
    def update_preset_custom(self, event=None):
        if self.preset_box.get() != "Manual Override":
            self.preset_box.set("Manual Override")
            self.apply_preset()

    def update_preview(self, event=None):
        if not hasattr(self, 'canvas') or self.canvas is None:
            return
        try:
            # Read variables safely to prevent division-by-zeros and float parsing issues
            speed_init = max(safe_float(self.speed_init_var.get(), 60.0), 1.0)
            speed_target = max(safe_float(self.speed_target_var.get(), 60.0), 1.0)
            t_speed_start = max(safe_float(self.speed_start_var.get(), 2.0), 0.0)
            t_speed_dur = max(safe_float(self.speed_dur_var.get(), 5.0), 0.0)
            
            t_end = max(safe_float(self.t_end_var.get(), 13.0), 1.0)
            maneuver = self.maneuver_box.get()
            t_start = max(safe_float(self.t_start_var.get(), 2.0), 0.0)
            t_dur = max(safe_float(self.t_dur_var.get(), 5.0), 1e-3)
            width = safe_float(self.width_var.get(), 5.0)
            dwell = max(safe_float(self.dwell_var.get(), 2.0), 0.0)
            
            radius = max(safe_float(self.radius_var.get(), 50.0), 1.0)
            direction = self.direction_box.get()
            lead_in = max(safe_float(self.lead_in_var.get(), 20.0), 0.0)
            
            radius2 = max(safe_float(self.radius2_var.get(), 20.0), 1.0)
            slalom_period = max(safe_float(self.slalom_period_var.get(), 30.0), 1.0)
            slalom_amp = safe_float(self.slalom_amp_var.get(), 2.0)
            theta1 = safe_float(self.theta1_var.get(), 45.0)
            theta2 = safe_float(self.theta2_var.get(), 180.0)
            
            clothoid_a = safe_float(self.clothoid_a_var.get(), 0.0005)
            vehicle_width = max(safe_float(self.vehicle_width_var.get(), 2.0), 0.1)

            start_margin = max(safe_float(self.start_margin_var.get(), 20.0), 0.0)
            end_margin = max(safe_float(self.end_margin_var.get(), 50.0), 0.0)
            
            slope_type = self.slope_type_box.get()
            slope_val = safe_float(self.slope_val_var.get(), 0.0) if slope_type != "None" else 0.0
            slope_start = safe_float(self.slope_start_var.get(), 0.0) if slope_type == "Smooth Step" else 0.0
            slope_dur = max(safe_float(self.slope_dur_var.get(), 0.0), 1e-3) if slope_type == "Smooth Step" else 0.0

            banking_type = self.banking_type_box.get()
            banking_val = safe_float(self.banking_val_var.get(), 0.0) if banking_type != "None" else 0.0
            banking_start = safe_float(self.banking_start_var.get(), 0.0) if banking_type == "Smooth Step" else 0.0
            banking_dur = max(safe_float(self.banking_dur_var.get(), 0.0), 1e-3) if banking_type == "Smooth Step" else 0.0

            # Compute Preview Geometry using Speed Profile Integration
            pts = get_preview_points(speed_init, speed_target, t_speed_start, t_speed_dur, t_end, maneuver, t_start, t_dur, width, dwell, radius, direction, lead_in, start_margin, radius2, slalom_period, slalom_amp, theta1, theta2, clothoid_a, vehicle_width)
            
            # Clear Canvas
            self.canvas.delete("all")
            
            # Query current canvas width dynamically (with fallback)
            canvas_width = self.canvas.winfo_width()
            if canvas_width < 10:
                canvas_width = 540 # Fallback default width
            
            # Draw Labels / Borders (Canvas expanded to 450px vertical height)
            self.canvas.create_rectangle(10, 15, canvas_width - 10, 145, outline="#27272a", width=1)
            self.canvas.create_text(20, 25, text="TOP-DOWN PATH PREVIEW (X-Y)", fill="#a1a1aa", anchor=tk.W, font=("Segoe UI", 7, "bold"))
            
            self.canvas.create_rectangle(10, 160, canvas_width - 10, 290, outline="#27272a", width=1)
            self.canvas.create_text(20, 170, text="ELEVATION SIDE PROFILE (Station-Z)", fill="#a1a1aa", anchor=tk.W, font=("Segoe UI", 7, "bold"))

            self.canvas.create_rectangle(10, 305, canvas_width - 10, 435, outline="#27272a", width=1)
            self.canvas.create_text(20, 315, text="BANK ANGLE PROFILE (Station-Banking)", fill="#a1a1aa", anchor=tk.W, font=("Segoe UI", 7, "bold"))

            # Plot region dimensions
            pw = canvas_width - 60
            ph = 90
            px_off, py_off1 = 30, 40
            py_off2 = 185
            py_off3 = 330
            
            # 1st Plot: Path Centerline
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            min_x, max_x = min(xs), max(xs)
            span_x = max_x - min_x
            if span_x < 1.0: span_x = 1.0
            
            max_y_bound = max(abs(min(ys)), abs(max(ys)), 2.0)
            
            # draw X axis reference lines
            self.canvas.create_line(px_off, py_off1 + ph/2, px_off + pw, py_off1 + ph/2, fill="#18181b", dash=(2, 2))
            
            path_pts = []
            for x, y in pts:
                px = px_off + (x - min_x) / span_x * pw
                py = (py_off1 + ph/2) - (y / max_y_bound) * (ph/2 - 10)
                path_pts.append((px, py))
                
            if len(path_pts) > 1:
                flat_path = [c for p in path_pts for c in p]
                self.canvas.create_line(*flat_path, fill="#6366f1", width=2)
                
            # Spawn Point Marker
            px_spawn = px_off + (start_margin - min_x) / span_x * pw
            py_spawn = py_off1 + ph/2
            self.canvas.create_line(px_spawn, py_off1 + 10, px_spawn, py_off1 + ph - 10, fill="#ef4444", dash=(4, 4))
            self.canvas.create_oval(px_spawn-4, py_spawn-4, px_spawn+4, py_spawn+4, fill="#ef4444", outline="#ffffff")
            self.canvas.create_text(px_spawn + 5, py_off1 + 15, text="Spawn", fill="#ef4444", anchor=tk.W, font=("Segoe UI", 7))

            # 2nd Plot: Slope Elevation
            v_init = speed_init / 3.6
            v_target = speed_target / 3.6
            
            def get_s_bound(t):
                if t_speed_dur <= 0:
                    return v_target * t + start_margin
                if t < t_speed_start:
                    return v_init * t + start_margin
                elif t < t_speed_start + t_speed_dur:
                    d1 = v_init * t_speed_start
                    dt2 = t - t_speed_start
                    v_end = v_init + (v_target - v_init) * (dt2 / t_speed_dur)
                    return d1 + 0.5 * (v_init + v_end) * dt2 + start_margin
                else:
                    d1 = v_init * t_speed_start
                    d2 = 0.5 * (v_init + v_target) * t_speed_dur
                    d3 = v_target * (t - t_speed_start - t_speed_dur)
                    return d1 + d2 + d3 + start_margin

            t_start_crg = -start_margin / v_init
            t_end_crg = t_end + end_margin / v_target
            u_length = max(get_s_bound(t_end_crg) - get_s_bound(t_start_crg), 1.0)
            
            slope_pts = get_slope_profile(u_length, slope_type, slope_val, slope_start, slope_dur)
            
            zs = [p[1] for p in slope_pts]
            min_z, max_z = min(zs), max(zs)
            span_z = max_z - min_z
            if span_z < 1.0:
                mid_z = 0.5 * (min_z + max_z)
                min_z = mid_z - 0.5
                max_z = mid_z + 0.5
                span_z = 1.0
                
            if min_z <= 0.0 <= max_z:
                py_zero = (py_off2 + ph - 15) - (0.0 - min_z) / span_z * (ph - 25)
                self.canvas.create_line(px_off, py_zero, px_off + pw, py_zero, fill="#18181b", dash=(2, 2))
                
            slope_poly = []
            for u, z in slope_pts:
                px = px_off + (u / u_length) * pw
                py = (py_off2 + ph - 15) - (z - min_z) / span_z * (ph - 25)
                slope_poly.append((px, py))
                
            if len(slope_poly) > 1:
                flat_slope = [c for p in slope_poly for c in p]
                self.canvas.create_line(*flat_slope, fill="#10b981", width=2)

            # 3rd Plot: Bank Angle Profile
            def get_banking_profile(u_len, b_type, b_val, b_start, b_dur, dense_pts):
                du_b = u_len / 100.0
                u_vals = [i * du_b for i in range(101)]
                b_profile = []
                
                # Approximate local curvature using central differences of path coordinates
                def get_local_curvature_at_u(u_val):
                    idx = int(min(max(0, u_val / u_len * (len(dense_pts)-1)), len(dense_pts)-1))
                    idx_m = max(0, idx - 2)
                    idx_p = min(len(dense_pts)-1, idx + 2)
                    if idx_p - idx_m < 4:
                        return 0.0
                    p_m = dense_pts[idx_m]
                    p_c = dense_pts[idx]
                    p_p = dense_pts[idx_p]
                    
                    d1 = np.sqrt((p_c[0] - p_m[0])**2 + (p_c[1] - p_m[1])**2)
                    d2 = np.sqrt((p_p[0] - p_c[0])**2 + (p_p[1] - p_c[1])**2)
                    if d1 < 1e-3 or d2 < 1e-3:
                        return 0.0
                    dx1 = (p_c[0] - p_m[0]) / d1
                    dy1 = (p_c[1] - p_m[1]) / d1
                    dx2 = (p_p[0] - p_c[0]) / d2
                    dy2 = (p_p[1] - p_c[1]) / d2
                    d2s = 0.5 * (d1 + d2)
                    d2s_safe = max(d2s, 1e-3)
                    return (dx1 * ((dy2 - dy1) / d2s_safe) - dy1 * ((dx2 - dx1) / d2s_safe))

                for u in u_vals:
                    if b_type == "None" or b_type == "Flat":
                        beta = 0.0
                    elif b_type == "Constant":
                        # Apply smooth 20m runout superelevation transition centered at s_turn_start
                        s_turn_start = start_margin + lead_in
                        beta = evaluate_profile(u, "Smooth Step", b_val, s_turn_start - 10.0, 20.0)
                    elif b_type == "Smooth Step":
                        beta = evaluate_profile(u, b_type, b_val, b_start, b_dur)
                    elif b_type == "Link to Curvature":
                        kappa = get_local_curvature_at_u(u)
                        scale = radius if maneuver in ["Circular Path", "J-Turn", "U-Turn", "Braking in a Turn", "Fishhook"] else 50.0
                        beta = -b_val * kappa * scale
                    else:
                        beta = 0.0
                    b_profile.append((u, beta))
                return b_profile

            bank_pts = get_banking_profile(u_length, banking_type, banking_val, banking_start, banking_dur, pts)
            betas = [p[1] for p in bank_pts]
            min_b, max_b = min(betas), max(betas)
            span_b = max_b - min_b
            if span_b < 0.02:
                mid_b = 0.5 * (min_b + max_b)
                min_b = mid_b - 0.01
                max_b = mid_b + 0.01
                span_b = 0.02
                
            if min_b <= 0.0 <= max_b:
                py_zero_b = (py_off3 + ph - 15) - (0.0 - min_b) / span_b * (ph - 25)
                self.canvas.create_line(px_off, py_zero_b, px_off + pw, py_zero_b, fill="#18181b", dash=(2, 2))
                
            bank_poly = []
            for u, b in bank_pts:
                px = px_off + (u / u_length) * pw
                py = (py_off3 + ph - 15) - (b - min_b) / span_b * (ph - 25)
                bank_poly.append((px, py))
                
            if len(bank_poly) > 1:
                flat_bank = [c for p in bank_poly for c in p]
                self.canvas.create_line(*flat_bank, fill="#6366f1", width=2)
                
            # Add scale metrics text
            self.canvas.create_text(px_off + pw - 5, py_off1 + ph - 5, text=f"Span-Y: {max_y_bound*2:.1f} m", fill="#71717a", anchor=tk.E, font=("Segoe UI", 7))
            self.canvas.create_text(px_off + pw - 5, py_off2 + ph - 5, text=f"Rise-Z: {max_z - min_z:.2f} m", fill="#71717a", anchor=tk.E, font=("Segoe UI", 7))
            self.canvas.create_text(px_off + pw - 5, py_off3 + ph - 5, text=f"Span-Bank: {max_b - min_b:.3f} rad", fill="#71717a", anchor=tk.E, font=("Segoe UI", 7))
            
        except Exception as e:
            print(f"Preview rendering exception: {e}")

    def start_generation(self):
        self.gen_btn.config(state=tk.DISABLED)
        self.log_text.delete(1.0, tk.END)
        self.save_settings()
        
        t = threading.Thread(target=self.run_generator)
        t.daemon = True
        t.start()

    def run_generator(self):
        try:
            # Parse parameters to pass to python API safely
            speed_init = safe_float(self.speed_init_var.get(), 60.0)
            speed_target = safe_float(self.speed_target_var.get(), 60.0)
            speed_start = safe_float(self.speed_start_var.get(), 2.0)
            speed_dur = safe_float(self.speed_dur_var.get(), 5.0)
            
            t_start = safe_float(self.t_start_var.get(), 2.0)
            t_dur = safe_float(self.t_dur_var.get(), 5.0)
            t_end = safe_float(self.t_end_var.get(), 13.0)
            width = safe_float(self.width_var.get(), 5.0)
            start_margin = safe_float(self.start_margin_var.get(), 20.0)
            end_margin = safe_float(self.end_margin_var.get(), 50.0)
            mesh_res = safe_float(self.mesh_res_var.get(), 0.06)
            r_width = safe_float(self.r_width_var.get(), 8.0)
            iso_class = self.iso_box.get().split(" ")[0]
            gen_obj = self.obj_val.get()
            
            maneuver = self.maneuver_box.get()
            dwell = safe_float(self.dwell_var.get(), 2.0)
            radius = safe_float(self.radius_var.get(), 50.0)
            direction = self.direction_box.get()
            lead_in = safe_float(self.lead_in_var.get(), 20.0)

            radius2 = safe_float(self.radius2_var.get(), 20.0)
            slalom_period = safe_float(self.slalom_period_var.get(), 30.0)
            slalom_amp = safe_float(self.slalom_amp_var.get(), 2.0)
            theta1 = safe_float(self.theta1_var.get(), 45.0)
            theta2 = safe_float(self.theta2_var.get(), 180.0)
            clothoid_a = safe_float(self.clothoid_a_var.get(), 0.0005)
            vehicle_width = safe_float(self.vehicle_width_var.get(), 2.0)
            
            slope_type = self.slope_type_box.get()
            slope_val = safe_float(self.slope_val_var.get(), 0.0)
            slope_start = safe_float(self.slope_start_var.get(), 30.0)
            slope_dur = safe_float(self.slope_dur_var.get(), 20.0)
            
            banking_type = self.banking_type_box.get()
            banking_val = safe_float(self.banking_val_var.get(), 0.0)
            banking_start = safe_float(self.banking_start_var.get(), 30.0)
            banking_dur = safe_float(self.banking_dur_var.get(), 20.0)
            
            mu_val = safe_float(self.mu_var.get(), 0.85)
            
            steering_type = 1 if self.steer_type_box.get() == "Stanley" else 0
            look_ahead = safe_float(self.look_ahead_var.get(), 3.615358)
            kp_steer = safe_float(self.kp_steer_var.get(), 2.398832)
            ki_steer = safe_float(self.ki_steer_var.get(), 0.0)
            kd_steer = safe_float(self.kd_steer_var.get(), 0.0)
            dead_zone = safe_float(self.dead_zone_var.get(), 0.010965)
            max_turn = safe_float(self.max_turn_var.get(), 25.0)
            
            kp_speed = safe_float(self.kp_speed_var.get(), 0.868900)
            ki_speed = safe_float(self.ki_speed_var.get(), 0.436516)
            kd_speed = safe_float(self.kd_speed_var.get(), 0.0)

            roughness_Nf = int(safe_float(self.roughness_nf_var.get(), 512))
            roughness_Ntheta = int(safe_float(self.roughness_nt_var.get(), 32))

            print("--- Executing Road Profile & Configuration Generation ---")
            generate_road_profile(
                target_speed_kph=speed_target,
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
                base_dir=SCRIPT_DIR,
                
                maneuver_type=maneuver,
                dwell_time=dwell,
                radius=radius,
                direction=direction,
                lead_in_length=lead_in,

                radius2=radius2,
                slalom_period=slalom_period,
                slalom_amplitude=slalom_amp,
                theta1=theta1,
                theta2=theta2,
                clothoid_a=clothoid_a,
                vehicle_width=vehicle_width,
                
                slope_type=slope_type,
                slope_value=slope_val,
                slope_start=slope_start,
                slope_duration=slope_dur,
                
                banking_type=banking_type,
                banking_value=banking_val,
                banking_start=banking_start,
                banking_duration=banking_dur,
                
                mu_value=mu_val,
                
                speed_profile_time_start=speed_start,
                speed_profile_duration=speed_dur,
                speed_profile_initial_speed=speed_init,
                speed_profile_target_speed=speed_target,
                
                steering_type=steering_type,
                look_ahead_dist=look_ahead,
                Kp_steering=kp_steer,
                Ki_steering=ki_steer,
                Kd_steering=kd_steer,
                stanley_dead_zone=dead_zone,
                max_wheel_turn_angle=max_turn,
                
                Kp_speed=kp_speed,
                Ki_speed=ki_speed,
                Kd_speed=kd_speed,

                roughness_Nf=roughness_Nf,
                roughness_Ntheta=roughness_Ntheta
            )
            print("\nSUCCESS: Path, CRG terrain, and simulation_parameters.m generated!")
            messagebox.showinfo("Success", "Road profile and Matlab configurations generated successfully!\n\nRun 'build_fmus.bat' afterwards to stage DLLs and resource files.")
        except Exception as e:
            print(f"\nERROR: Generation failed. {e}")
            messagebox.showerror("Generation Failed", f"An error occurred: {e}")
        finally:
            self.gen_btn.config(state=tk.NORMAL)
    def on_window_resize(self, event):
        if event.widget == self.root:
            self.update_preview()

    def process_log_queue(self):
        while True:
            try:
                msg = self.log_queue.get_nowait()
                self.log_text.insert(tk.END, msg)
                self.log_text.see(tk.END)
            except queue.Empty:
                break
        self.root.after(100, self.process_log_queue)

if __name__ == "__main__":
    root = tk.Tk()
    app = RoadGeneratorGUI(root)
    root.mainloop()
