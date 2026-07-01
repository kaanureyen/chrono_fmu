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
def get_preview_points(speed_init, speed_target, t_speed_start, t_speed_dur, t_end, maneuver, t_start, t_dur, width, dwell, radius, direction, lead_in, start_margin, end_margin, radius2=20.0, slalom_period=30.0, slalom_amplitude=2.0, theta1=45.0, theta2=180.0, clothoid_a=0.0005, vehicle_width=2.0, superpose_lc=False, lc_start_time=2.0, lc_duration=5.0, lc_width=5.0):
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
        "y_clothoid": y_clothoid,
        "superpose_lc": superpose_lc,
        "lc_start_time": lc_start_time,
        "lc_duration": lc_duration,
        "lc_width": lc_width
    }
 
    for t in times:
        x, y, phi = get_trajectory_pose(t, params)
        points.append((x, y))
        
    # Prepend start margin (straight line along X axis)
    if start_margin > 0.0:
        start_pts = [(start_margin * i / 5.0, 0.0) for i in range(5)]
        points = start_pts + points
        
    # Append end margin (tangent extension from the last point)
    if end_margin > 0.0 and len(points) >= 2:
        x_last, y_last = points[-1]
        x_pen, y_pen = points[-2]
        dx = x_last - x_pen
        dy = y_last - y_pen
        len_d = np.sqrt(dx**2 + dy**2)
        if len_d > 1e-5:
            dx /= len_d
            dy /= len_d
        else:
            dx, dy = 1.0, 0.0
        for i in range(1, 6):
            dist = end_margin * i / 5.0
            points.append((x_last + dist * dx, y_last + dist * dy))
        
    return points

def get_slope_profile(u_length, slope_type, slope_val, slope_start, slope_dur):
    du = u_length / 100.0
    u_vals = [i * du for i in range(101)]
    z_vals = [0.0]
    
    for i in range(100):
        s_current = evaluate_profile(u_vals[i], slope_type, slope_val, slope_start, slope_dur)
        z_vals.append(z_vals[-1] + du * s_current)
        
    return list(zip(u_vals, z_vals))

def resolve_texture_path(tex_name):
    if not tex_name or tex_name == "None":
        return None
    script_dir = os.path.dirname(os.path.abspath(__file__))
    chrono_build_dir = os.path.abspath(os.path.join(script_dir, "..", "chrono_build"))
    chrono_data_dir = os.path.join(chrono_build_dir, "data")
    
    if tex_name == "road.jpg":
        return os.path.join(chrono_data_dir, "textures", "road.jpg")
    elif tex_name == "concrete.jpg":
        return os.path.join(chrono_data_dir, "textures", "concrete.jpg")
    elif tex_name == "rock.jpg":
        return os.path.join(chrono_data_dir, "textures", "rock.jpg")
    elif tex_name == "dirt.jpg":
        return os.path.join(chrono_data_dir, "vehicle", "terrain", "textures", "dirt.jpg")
    elif tex_name == "concrete_color.jpg":
        return os.path.join(chrono_data_dir, "vehicle", "terrain", "textures", "Concrete002_2K-JPG", "Concrete002_2K_Color.jpg")
    return None

class RoadGeneratorGUI:
    # Maneuver Defaults Dictionary for GUI reset
    MANEUVER_DEFAULTS = {
        "Straight Line": {},
        "Single Lane Change": {"t_start": 2.0, "t_duration": 5.0, "width": 5.0, "t_end": 13.0, "speed_profile_initial_speed": 70.0, "speed_profile_target_speed": 70.0},
        "Double Lane Change": {"t_start": 2.0, "t_duration": 5.0, "width": 5.0, "dwell_time": 2.0, "t_end": 15.0, "speed_profile_initial_speed": 70.0, "speed_profile_target_speed": 70.0},
        "Circular Path": {"radius": 50.0, "direction": "Left", "lead_in_length": 20.0, "t_end": 25.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "J-Turn": {"radius": 50.0, "direction": "Left", "lead_in_length": 20.0, "t_end": 15.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "U-Turn": {"radius": 50.0, "direction": "Left", "lead_in_length": 20.0, "t_end": 20.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "Slalom": {"lead_in_length": 20.0, "slalom_period": 36.0, "slalom_amplitude": 1.5, "t_end": 20.0, "speed_profile_initial_speed": 60.0, "speed_profile_target_speed": 60.0},
        "Sine with Dwell": {"t_start": 2.0, "t_duration": 1.43, "width": 3.5, "dwell_time": 0.5, "t_end": 10.0, "speed_profile_initial_speed": 80.0, "speed_profile_target_speed": 80.0},
        "Fishhook": {"radius": 50.0, "radius2": 20.0, "direction": "Left", "lead_in_length": 20.0, "theta1": 45.0, "theta2": 180.0, "t_end": 15.0, "speed_profile_initial_speed": 80.0, "speed_profile_target_speed": 80.0},
        "ISO 3888-2 Obstacle Avoidance": {"lead_in_length": 20.0, "vehicle_width": 2.0, "t_end": 15.0, "speed_profile_initial_speed": 70.0, "speed_profile_target_speed": 70.0},
        "Constant Speed Spiral": {"direction": "Left", "lead_in_length": 20.0, "clothoid_a": 0.0005, "t_end": 15.0, "speed_profile_initial_speed": 50.0, "speed_profile_target_speed": 50.0},
        "Braking in a Turn": {"radius": 100.0, "direction": "Left", "lead_in_length": 20.0, "speed_profile_initial_speed": 80.0, "speed_profile_target_speed": 0.0, "speed_profile_time_start": 3.0, "speed_profile_duration": 2.0, "t_end": 8.0}
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
                "banking_start": 0.0, # Obsolete parameter, kept for compatibility
                "banking_duration": 0.0, # Obsolete parameter, kept for compatibility
                "curvature_filter_window_m": safe_float(self.curv_filter_var.get(), 15.0),
                
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
                "roughness_Ntheta": int(safe_float(self.roughness_nt_var.get(), 32)),

                # Visual, packaging, and superposition parameters
                "superpose_lc": self.superpose_lc_val.get(),
                "lc_start_time": safe_float(self.lc_start_var.get(), 2.0),
                "lc_duration": safe_float(self.lc_dur_var.get(), 5.0),
                "lc_width": safe_float(self.lc_width_var.get(), 5.0),
                "terrain_diffuse_texture": "" if self.tex_diffuse_var.get() == "None" else self.tex_diffuse_var.get(),
                "terrain_normal_texture": "" if self.tex_normal_var.get() == "None" else self.tex_normal_var.get(),
                "terrain_show_visual_lines": self.show_lines_val.get(),
                "terrain_crg_simplify": self.simplify_mesh_val.get(),
                "vehicle_visible": self.vehicle_vis_val.get(),
                "driver_visible": self.driver_vis_val.get(),
                "terrain_type": self.terrain_type_box.get()
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

    def add_grid_entry(self, parent, row, col, label_text, default_val, tooltip_text=None):
        lbl_frame = tk.Frame(parent, bg="#121214")
        lbl_frame.grid(row=row, column=col, sticky=tk.W, pady=2, padx=4)
        
        lbl = ttk.Label(lbl_frame, text=label_text, font=("Segoe UI", 8))
        lbl.pack(side=tk.LEFT)
        
        if tooltip_text:
            info_lbl = tk.Label(lbl_frame, text="ⓘ", bg="#121214", fg="#6366f1", font=("Segoe UI", 9, "bold"), cursor="hand2")
            info_lbl.pack(side=tk.LEFT, padx=(3, 0))
            ToolTip(info_lbl, tooltip_text)

        entry = ttk.Entry(parent, font=("Segoe UI", 8), width=10)
        entry.grid(row=row, column=col+1, sticky=tk.EW, pady=2, padx=4)
        entry.insert(0, str(default_val))
        return entry

    def add_grid_combobox(self, parent, row, col, label_text, values, default_val, callback=None, tooltip_text=None):
        lbl_frame = tk.Frame(parent, bg="#121214")
        lbl_frame.grid(row=row, column=col, sticky=tk.W, pady=2, padx=4)
        
        lbl = ttk.Label(lbl_frame, text=label_text, font=("Segoe UI", 8))
        lbl.pack(side=tk.LEFT)
        
        if tooltip_text:
            info_lbl = tk.Label(lbl_frame, text="ⓘ", bg="#121214", fg="#6366f1", font=("Segoe UI", 9, "bold"), cursor="hand2")
            info_lbl.pack(side=tk.LEFT, padx=(3, 0))
            ToolTip(info_lbl, tooltip_text)

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
        tab_telemetry = tk.Frame(self.notebook, bg="#121214")

        self.notebook.add(tab_maneuver, text=" Scenario & Maneuver ")
        self.notebook.add(tab_road, text=" Road & Environment ")
        self.notebook.add(tab_controller, text=" Controller Gains ")
        self.notebook.add(tab_telemetry, text=" Telemetry & Simulation ")

        # Build Tab 1
        self.build_maneuver_tab(tab_maneuver)

        # Build Tab 2
        self.build_road_tab(tab_road)

        # Build Tab 3
        self.build_controller_tab(tab_controller)
        
        # Build Tab 4
        self.build_telemetry_tab(tab_telemetry)

        # Build Right Panel (Canvas + Log Output)
        self.build_right_panel(right_panel)

    def build_maneuver_tab(self, parent):
        # Card 1: Scenario Configuration
        scen_frame = ttk.LabelFrame(parent, text=" 1. Scenario & Speed Configuration ")
        scen_frame.pack(fill=tk.X, pady=5)
        
        self.maneuver_box = self.add_combobox_field(scen_frame, 0, "Maneuver Type:", 
            ["Straight Line", "Single Lane Change", "Double Lane Change", "Circular Path", "J-Turn", "U-Turn", "Slalom", "Sine with Dwell", "Fishhook", "ISO 3888-2 Obstacle Avoidance", "Constant Speed Spiral", "Braking in a Turn"],
            self.loaded_params.get("maneuver_type", "Single Lane Change"), self.update_maneuver_visibility,
            tooltip_text="The type of vehicle dynamic maneuver to simulate (Straight, Slalom, Lane Change, etc.).")
        
        self.t_end_var = self.add_entry_field(scen_frame, 1, "Simulation Stop Time (s):", self.loaded_params.get("t_end", 13.0),
            tooltip_text="Total simulation stop time (in seconds).")
        
        self.speed_init_var = self.add_entry_field(scen_frame, 2, "Initial Speed (km/h):", self.loaded_params.get("speed_profile_initial_speed", 70.0),
            tooltip_text="Initial velocity of the vehicle at simulation spawn (in km/h).")
        self.speed_target_var = self.add_entry_field(scen_frame, 3, "Target Speed (km/h):", self.loaded_params.get("speed_profile_target_speed", 70.0),
            tooltip_text="Target cruise speed of the vehicle (in km/h).")
        self.speed_start_var = self.add_entry_field(scen_frame, 4, "Speed Ramp Start Time (s):", self.loaded_params.get("speed_profile_time_start", 2.0),
            tooltip_text="Time (in seconds) when the speed ramp controller starts to accelerate or decelerate.")
        self.speed_dur_var = self.add_entry_field(scen_frame, 5, "Speed Ramp Duration (s):", self.loaded_params.get("speed_profile_duration", 5.0),
            tooltip_text="Duration (in seconds) over which the vehicle speed ramps from initial to target speed.")

        # Card 2: Maneuver Details (Static grid layout to display all parameters cleanly)
        self.man_frame = ttk.LabelFrame(parent, text=" 2. Maneuver Parameters ")
        self.man_frame.pack(fill=tk.X, pady=5)

        self.man_frame.columnconfigure(0, weight=1)
        self.man_frame.columnconfigure(1, weight=2)
        self.man_frame.columnconfigure(2, weight=1)
        self.man_frame.columnconfigure(3, weight=2)

        # Row 0: Start Time, Transition Dur
        self.t_start_var = self.add_grid_entry(self.man_frame, 0, 0, "Maneuver Start (s):", self.loaded_params.get("t_start", 2.0),
            tooltip_text="Time (in seconds) after the simulation start when the active vehicle steering maneuver begins.")
        self.t_dur_var = self.add_grid_entry(self.man_frame, 0, 2, "Transition (s):", self.loaded_params.get("t_duration", 5.0),
            tooltip_text="Duration of the entry/exit transition (in seconds) for lane changes or slalom steer inputs.")

        # Row 1: Lateral Offset, Dwell Time
        self.width_var = self.add_grid_entry(self.man_frame, 1, 0, "Lateral Offset (m):", self.loaded_params.get("width", 5.0),
            tooltip_text="Target lateral displacement (in meters) for lane changes or obstacle avoidance.")
        self.dwell_var = self.add_grid_entry(self.man_frame, 1, 2, "Dwell Time (s):", self.loaded_params.get("dwell_time", 2.0),
            tooltip_text="Time (in seconds) the vehicle spends at the maximum lateral displacement or between turns.")

        # Row 2: Circular Radius, Turn Direction
        self.radius_var = self.add_grid_entry(self.man_frame, 2, 0, "Circle Radius (m):", self.loaded_params.get("radius", 50.0),
            tooltip_text="Radius (in meters) of the reference circular path center line.")
        self.direction_box = self.add_grid_combobox(self.man_frame, 2, 2, "Direction:", ["Left", "Right"], self.loaded_params.get("direction", "Left"),
            tooltip_text="Turn direction (Left or Right) for curved maneuvers.")

        # Row 3: Lead-in Length, Vehicle Width
        self.lead_in_var = self.add_grid_entry(self.man_frame, 3, 0, "Lead-in Len (m):", self.loaded_params.get("lead_in_length", 20.0),
            tooltip_text="Length (in meters) of straight tangent track before starting the curved section.")
        self.vehicle_width_var = self.add_grid_entry(self.man_frame, 3, 2, "Vehicle Width (m):", self.loaded_params.get("vehicle_width", 2.0),
            tooltip_text="Width of the vehicle (in meters) used to scale the lane definitions according to ISO 3888 specifications.")

        # Row 4: Slalom Period, Slalom Amp
        self.slalom_period_var = self.add_grid_entry(self.man_frame, 4, 0, "Slalom Period (m):", self.loaded_params.get("slalom_period", 30.0),
            tooltip_text="Wavelength/period (in meters) of the slalom path sine wave.")
        self.slalom_amp_var = self.add_grid_entry(self.man_frame, 4, 2, "Slalom Amp (m):", self.loaded_params.get("slalom_amplitude", 2.0),
            tooltip_text="Amplitude (lateral deviation in meters) of the slalom path.")

        # Row 5: Fishhook Radius 2, Clothoid Param A
        self.radius2_var = self.add_grid_entry(self.man_frame, 5, 0, "Fishhook Rad 2 (m):", self.loaded_params.get("radius2", 20.0),
            tooltip_text="Radius of the second counter-steering turn (in meters) for NCAP fishhook maneuver.")
        self.clothoid_a_var = self.add_grid_entry(self.man_frame, 5, 2, "Clothoid Param A:", self.loaded_params.get("clothoid_a", 0.0005),
            tooltip_text="Scaling parameter A (in 1/m) of the spiral transition curve.")

        # Row 6: Fishhook Angle 1, Fishhook Angle 2
        self.theta1_var = self.add_grid_entry(self.man_frame, 6, 0, "Turn 1 Angle (deg):", self.loaded_params.get("theta1", 45.0),
            tooltip_text="Target steer or heading angle (in degrees) for the first turn of the fishhook maneuver.")
        self.theta2_var = self.add_grid_entry(self.man_frame, 6, 2, "Turn 2 Angle (deg):", self.loaded_params.get("theta2", 180.0),
            tooltip_text="Target steer or heading angle (in degrees) for the second turn of the fishhook maneuver.")

        # Row 7: Reset Button
        self.reset_btn = ttk.Button(self.man_frame, text="Reset to Maneuver Defaults", command=self.reset_maneuver_defaults)
        self.reset_btn.grid(row=7, column=0, columnspan=4, pady=8, padx=10, sticky=tk.EW)
        
        # Card 3: Lane Change Superposition
        lc_frame = ttk.LabelFrame(parent, text=" 3. Lane Change Superposition ")
        lc_frame.pack(fill=tk.X, pady=5)
        
        lc_frame.columnconfigure(0, weight=1)
        lc_frame.columnconfigure(1, weight=2)
        lc_frame.columnconfigure(2, weight=1)
        lc_frame.columnconfigure(3, weight=2)
        
        self.superpose_lc_val = tk.BooleanVar(value=self.loaded_params.get("superpose_lc", False))
        self.superpose_lc_cb = ttk.Checkbutton(lc_frame, text="Enable Superposed Lane Change", variable=self.superpose_lc_val, command=self.update_preview)
        self.superpose_lc_cb.grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=4, padx=10)
        
        self.lc_start_var = self.add_grid_entry(lc_frame, 1, 0, "LC Start Time (s):", self.loaded_params.get("lc_start_time", 2.0),
            tooltip_text="Start time (in seconds) of the superposed lane change maneuver.")
        self.lc_dur_var = self.add_grid_entry(lc_frame, 1, 2, "LC Duration (s):", self.loaded_params.get("lc_duration", 5.0),
            tooltip_text="Duration (in seconds) of the superposed lane change transition.")
        self.lc_width_var = self.add_grid_entry(lc_frame, 2, 0, "LC Width (m):", self.loaded_params.get("lc_width", 5.0),
            tooltip_text="Lateral offset width (in meters) of the superposed lane change.")

    def build_road_tab(self, parent):
        # Card 0: Road Layout Dimensions & Margins
        layout_frame = ttk.LabelFrame(parent, text=" 1. Road Dimensions & Buffer Margins ")
        layout_frame.pack(fill=tk.X, pady=5)
        
        self.start_margin_var = self.add_entry_field(layout_frame, 0, "Start Margin (m):", self.loaded_params.get("start_length_margin", 20.0),
            tooltip_text="Buffer zone of straight flat track before the active maneuver to ensure simulation stability.")
        self.end_margin_var = self.add_entry_field(layout_frame, 1, "End Margin (m):", self.loaded_params.get("end_length_margin", 50.0),
            tooltip_text="Buffer zone of straight flat track after the active maneuver to ensure vehicle deceleration and stabilization.")
        self.r_width_var = self.add_entry_field(layout_frame, 2, "Road Width (m):", self.loaded_params.get("v_width", 8.0),
            tooltip_text="Total lateral width (in meters) of the generated road surface.")

        # Card 1: Road Surface presets
        road_frame = ttk.LabelFrame(parent, text=" 2. Road Surface & Friction ")
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
        sb_frame = ttk.LabelFrame(parent, text=" 3. Road Slope & Banking ")
        sb_frame.pack(fill=tk.X, pady=5)
        
        self.slope_type_box = self.add_combobox_field(sb_frame, 0, "Slope Profile Type:", ["None", "Constant", "Smooth Step"], self.loaded_params.get("slope_type", "None"), self.update_slope_visibility)
        self.slope_val_var = self.add_entry_field(sb_frame, 1, "Slope Value (m/m):", self.loaded_params.get("slope_value", 0.02))
        self.slope_start_var = self.add_entry_field(sb_frame, 2, "Slope Start Station (m):", self.loaded_params.get("slope_start", 30.0))
        self.slope_dur_var = self.add_entry_field(sb_frame, 3, "Slope Transition Dist (m):", self.loaded_params.get("slope_duration", 20.0))
        
        self.banking_type_box = self.add_combobox_field(sb_frame, 4, "Banking Profile Type:", ["None", "Link to Curvature", "Balance Lateral Acceleration"], self.loaded_params.get("banking_type", "None"), self.update_banking_visibility)
        self.banking_val_var = self.add_entry_field(sb_frame, 5, "Banking Value (m/m):", self.loaded_params.get("banking_value", 0.04))
        self.curv_filter_var = self.add_entry_field(sb_frame, 6, "Curvature Filter Window (m):", self.loaded_params.get("curvature_filter_window_m", 15.0))
        
        # Card 3: FMU Visual & Packaging Settings
        vis_frame = ttk.LabelFrame(parent, text=" 4. FMU Visual & Packaging Settings ")
        vis_frame.pack(fill=tk.X, pady=5)
        
        self.terrain_type_box = self.add_combobox_field(vis_frame, 0, "Terrain Type:", ["OpenCRG", "OBJ Mesh"], self.loaded_params.get("terrain_type", "OpenCRG"), self.update_preview)

        diffuse_opts = ["None", "concrete.jpg", "concrete_color.jpg", "dirt.jpg", "road.jpg", "rock.jpg"]
        saved_diffuse = self.loaded_params.get("terrain_diffuse_texture", "road.jpg")
        if not saved_diffuse:
            saved_diffuse = "None"
        self.tex_diffuse_var = self.add_combobox_field(vis_frame, 1, "Diffuse Texture:", diffuse_opts, saved_diffuse, self.update_preview)
        
        normal_opts = ["None", "concrete_normal.jpg"]
        saved_normal = self.loaded_params.get("terrain_normal_texture", "")
        if not saved_normal:
            saved_normal = "None"
        self.tex_normal_var = self.add_combobox_field(vis_frame, 2, "Normal Texture:", normal_opts, saved_normal, self.update_preview)
        
        self.show_lines_val = tk.BooleanVar(value=self.loaded_params.get("terrain_show_visual_lines", True))
        self.show_lines_cb = ttk.Checkbutton(vis_frame, text="Draw Visual Lane Markings (White/Yellow Lines)", variable=self.show_lines_val, command=self.update_preview)
        self.show_lines_cb.grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=4, padx=10)
        
        self.simplify_mesh_val = tk.BooleanVar(value=self.loaded_params.get("terrain_crg_simplify", True))
        self.simplify_mesh_cb = ttk.Checkbutton(vis_frame, text="Simplify OpenCRG Visualization Mesh", variable=self.simplify_mesh_val, command=self.update_preview)
        self.simplify_mesh_cb.grid(row=4, column=0, columnspan=2, sticky=tk.W, pady=4, padx=10)

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
        # 1. Preview Notebook (split into Profiles and 1:1 tabs)
        self.preview_notebook = ttk.Notebook(parent, style="TNotebook")
        self.preview_notebook.pack(fill=tk.X, pady=5)
        
        # Tab 1: Profiles
        tab_profiles = tk.Frame(self.preview_notebook, bg="#121214")
        self.preview_notebook.add(tab_profiles, text=" Profiles & Path Preview ")
        self.canvas = tk.Canvas(tab_profiles, bg="#09090b", height=450, bd=0, relief=tk.SOLID, highlightthickness=0)
        self.canvas.pack(fill=tk.X, expand=True)
        
        # Tab 2: 1:1 Real-Scale Path
        tab_realscale = tk.Frame(self.preview_notebook, bg="#121214")
        self.preview_notebook.add(tab_realscale, text=" 1:1 Real-Scale Path ")
        self.canvas_realscale = tk.Canvas(tab_realscale, bg="#09090b", height=450, bd=0, relief=tk.SOLID, highlightthickness=0)
        self.canvas_realscale.pack(fill=tk.X, expand=True)
 
        # 2. Action Buttons Frame
        btn_frame = tk.Frame(parent, bg="#121214")
        btn_frame.pack(fill=tk.X, pady=10)
        
        self.gen_btn = ttk.Button(btn_frame, text="GENERATE ROAD & CONFIG", style="Accent.TButton", command=self.start_generation)
        self.gen_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        self.view3d_btn = ttk.Button(btn_frame, text="3D CAD VIEWER", command=self.start_3d_viewer)
        self.view3d_btn.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(5, 0))

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

    def add_entry_field(self, parent, row, label_text, default_val, tooltip_text=None):
        lbl_frame = tk.Frame(parent, bg="#121214")
        lbl_frame.grid(row=row, column=0, sticky=tk.W, pady=1, padx=10)
        
        lbl = ttk.Label(lbl_frame, text=label_text)
        lbl.pack(side=tk.LEFT)
        
        if tooltip_text:
            info_lbl = tk.Label(lbl_frame, text="ⓘ", bg="#121214", fg="#6366f1", font=("Segoe UI", 9, "bold"), cursor="hand2")
            info_lbl.pack(side=tk.LEFT, padx=(3, 0))
            ToolTip(info_lbl, tooltip_text)

        entry = ttk.Entry(parent, font=("Segoe UI", 9))
        entry.grid(row=row, column=1, sticky=tk.EW, pady=1, padx=10)
        entry.insert(0, str(default_val))
        parent.columnconfigure(1, weight=1)
        return entry

    def add_combobox_field(self, parent, row, label_text, values, default_val, callback=None, tooltip_text=None):
        lbl_frame = tk.Frame(parent, bg="#121214")
        lbl_frame.grid(row=row, column=0, sticky=tk.W, pady=1, padx=10)
        
        lbl = ttk.Label(lbl_frame, text=label_text)
        lbl.pack(side=tk.LEFT)
        
        if tooltip_text:
            info_lbl = tk.Label(lbl_frame, text="ⓘ", bg="#121214", fg="#6366f1", font=("Segoe UI", 9, "bold"), cursor="hand2")
            info_lbl.pack(side=tk.LEFT, padx=(3, 0))
            ToolTip(info_lbl, tooltip_text)

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
                         "slope_start_var", "slope_dur_var", "banking_val_var", "curv_filter_var",
                         "lc_start_var", "lc_dur_var", "lc_width_var",
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
            self.curv_filter_var.config(state=tk.DISABLED)
        elif bt == "Link to Curvature":
            self.banking_val_var.config(state=tk.NORMAL)
            self.curv_filter_var.config(state=tk.NORMAL)
        elif bt == "Balance Lateral Acceleration":
            self.banking_val_var.config(state=tk.NORMAL)
            self.curv_filter_var.config(state=tk.NORMAL)
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
            # Toggle simplify_mesh checkbutton state based on terrain type
            if self.terrain_type_box.get() == "OpenCRG":
                self.simplify_mesh_cb.config(state=tk.NORMAL)
            else:
                self.simplify_mesh_cb.config(state=tk.DISABLED)

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
            curv_filter = safe_float(self.curv_filter_var.get(), 0.0)
            banking_start = 0.0
            banking_dur = 0.0
 
            # Read superposition variables
            superpose_lc = self.superpose_lc_val.get()
            lc_start = safe_float(self.lc_start_var.get(), 2.0)
            lc_dur = safe_float(self.lc_dur_var.get(), 5.0)
            lc_width = safe_float(self.lc_width_var.get(), 5.0)
 
            # Compute Preview Geometry using Speed Profile Integration
            pts = get_preview_points(speed_init, speed_target, t_speed_start, t_speed_dur, t_end, maneuver, t_start, t_dur, width, dwell, radius, direction, lead_in, start_margin, end_margin, radius2, slalom_period, slalom_amp, theta1, theta2, clothoid_a, vehicle_width, superpose_lc, lc_start, lc_dur, lc_width)
            
            # Clear Canvas
            self.canvas.delete("all")
            
            # Reconstruct variables for 1:1 scaling
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            min_x, max_x = min(xs), max(xs)
            span_x = max_x - min_x
            if span_x < 1.0: span_x = 1.0
            
            # Draw 1:1 Real-Scale Path
            if hasattr(self, 'canvas_realscale') and self.canvas_realscale is not None:
                self.canvas_realscale.delete("all")
                canvas_w_r = self.canvas_realscale.winfo_width()
                if canvas_w_r < 10:
                    canvas_w_r = 540
                
                pw_r = canvas_w_r - 60
                ph_r = 390
                px_off_r = 30
                py_off_r = 30
                
                min_y, max_y = min(ys), max(ys)
                span_y = max_y - min_y
                if span_y < 1.0: span_y = 1.0
                
                scale_x = pw_r / span_x
                scale_y = ph_r / span_y
                scale = min(scale_x, scale_y)
                
                x_center_plot = px_off_r + pw_r / 2.0
                y_center_plot = py_off_r + ph_r / 2.0
                x_center_data = (min_x + max_x) / 2.0
                y_center_data = (min_y + max_y) / 2.0
                
                self.canvas_realscale.create_rectangle(10, 15, canvas_w_r - 10, 435, outline="#27272a", width=1)
                self.canvas_realscale.create_text(20, 25, text="1:1 REAL-SCALE TOP-DOWN PATH (PROPORTIONAL X-Y)", fill="#a1a1aa", anchor=tk.W, font=("Segoe UI", 7, "bold"))
                
                path_pts_r = []
                for x, y in pts:
                    px = x_center_plot + (x - x_center_data) * scale
                    py = y_center_plot - (y - y_center_data) * scale
                    path_pts_r.append((px, py))
                    
                if len(path_pts_r) > 1:
                    flat_path_r = [c for p in path_pts_r for c in p]
                    self.canvas_realscale.create_line(*flat_path_r, fill="#10b981", width=2)
                    
                px_spawn_r = x_center_plot + (start_margin - x_center_data) * scale
                py_spawn_r = y_center_plot - (0.0 - y_center_data) * scale
                self.canvas_realscale.create_oval(px_spawn_r-4, py_spawn_r-4, px_spawn_r+4, py_spawn_r+4, fill="#ef4444", outline="#ffffff")
                self.canvas_realscale.create_text(px_spawn_r + 5, py_spawn_r + 5, text="Spawn", fill="#ef4444", anchor=tk.W, font=("Segoe UI", 7))
            
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
                
                # Setup speed profile variables (converted from km/h to m/s)
                v_init = speed_init / 3.6
                v_target = speed_target / 3.6
                s_speed_start = v_init * t_speed_start + start_margin
                v_avg = 0.5 * (v_init + v_target)
                s_speed_dur = v_avg * t_speed_dur
                
                def get_speed_at_u(u_val):
                    if t_speed_dur <= 0.0 or s_speed_dur <= 0.0:
                        return v_target
                    if u_val < s_speed_start:
                        return v_init
                    elif u_val > s_speed_start + s_speed_dur:
                        return v_target
                    else:
                        tau = (u_val - s_speed_start) / s_speed_dur
                        s_val = 3.0 * tau**2 - 2.0 * tau**3
                        return v_init + (v_target - v_init) * s_val
                
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
 
                kappas = [get_local_curvature_at_u(u) for u in u_vals]
                if curv_filter > 0.0:
                    window_size = int(round(curv_filter / du_b))
                    if window_size > 1:
                        window = np.ones(window_size) / window_size
                        padded = np.pad(kappas, (window_size//2, window_size - 1 - window_size//2), mode='edge')
                        kappas = np.convolve(padded, window, mode='valid')

                for idx, u in enumerate(u_vals):
                    kappa = kappas[idx]
                    if b_type == "None" or b_type == "Flat":
                        beta = 0.0
                    elif b_type == "Constant":
                        # Apply smooth 20m runout superelevation transition centered at s_turn_start
                        s_turn_start = start_margin + lead_in
                        beta = evaluate_profile(u, "Smooth Step", b_val, s_turn_start - 10.0, 20.0)
                    elif b_type == "Smooth Step":
                        beta = evaluate_profile(u, b_type, b_val, b_start, b_dur)
                    elif b_type == "Link to Curvature":
                        scale = radius if maneuver in ["Circular Path", "J-Turn", "U-Turn", "Braking in a Turn", "Fishhook"] else 50.0
                        beta = -b_val * kappa * scale
                    elif b_type == "Balance Lateral Acceleration":
                        v_curr = get_speed_at_u(u)
                        # Convert tan_beta to sin_beta to match the new C++ generator projection math
                        tan_beta = - (v_curr**2 * kappa) / 9.80665
                        sin_beta = tan_beta / np.sqrt(1.0 + tan_beta**2)
                        sin_limit = abs(b_val) / np.sqrt(1.0 + b_val**2)
                        beta = np.clip(sin_beta, -sin_limit, sin_limit)
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
            self.canvas.create_text(px_off + pw - 5, py_off3 + ph - 5, text=f"Span-Bank: {max_b - min_b:.3f} m/m", fill="#71717a", anchor=tk.E, font=("Segoe UI", 7))
            
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
            
            curv_filter_w = safe_float(self.curv_filter_var.get(), 15.0)
            superpose_lc = self.superpose_lc_val.get()
            lc_start = safe_float(self.lc_start_var.get(), 2.0)
            lc_dur = safe_float(self.lc_dur_var.get(), 5.0)
            lc_width = safe_float(self.lc_width_var.get(), 5.0)
            diffuse_tex = "" if self.tex_diffuse_var.get() == "None" else self.tex_diffuse_var.get()
            normal_tex = "" if self.tex_normal_var.get() == "None" else self.tex_normal_var.get()
            show_lines = self.show_lines_val.get()
            simplify_mesh = self.simplify_mesh_val.get()
 
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
                banking_start=0.0,
                banking_duration=0.0,
                curvature_filter_window_m=curv_filter_w,
                
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
                roughness_Ntheta=roughness_Ntheta,
                
                superpose_lc=superpose_lc,
                lc_start_time=lc_start,
                lc_duration=lc_dur,
                lc_width=lc_width,
                terrain_diffuse_texture=diffuse_tex,
                terrain_normal_texture=normal_tex,
                terrain_show_visual_lines=show_lines,
                terrain_crg_simplify=simplify_mesh
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

    def build_telemetry_tab(self, parent):
        # Card 1: Co-Simulation Execution
        run_frame = ttk.LabelFrame(parent, text=" 1. Co-Simulation Execution ")
        run_frame.pack(fill=tk.X, pady=5)
        
        self.vehicle_vis_val = tk.BooleanVar(value=self.loaded_params.get("vehicle_visible", True))
        self.vehicle_vis_cb = ttk.Checkbutton(run_frame, text="Enable Vehicle 3D Window (Chrono View)", variable=self.vehicle_vis_val)
        self.vehicle_vis_cb.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=4, padx=10)
        
        self.driver_vis_val = tk.BooleanVar(value=self.loaded_params.get("driver_visible", False))
        self.driver_vis_cb = ttk.Checkbutton(run_frame, text="Enable Driver Sentinel Window (Reference Preview)", variable=self.driver_vis_val)
        self.driver_vis_cb.grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=4, padx=10)
        
        self.run_sim_btn = ttk.Button(run_frame, text="RUN CO-SIMULATION", style="Accent.TButton", command=self.start_simulation)
        self.run_sim_btn.grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=10, padx=10)
        
        self.sim_status_lbl = ttk.Label(run_frame, text="Status: Ready to run", foreground="#a1a1aa", font=("Segoe UI", 9, "italic"))
        self.sim_status_lbl.grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=5, padx=10)
        
        # Card 2: Telemetry Plots
        plot_frame = ttk.LabelFrame(parent, text=" 2. Telemetry Plots ")
        plot_frame.pack(fill=tk.X, pady=5)
        
        self.plot_traj_btn = ttk.Button(plot_frame, text="Plot Vehicle Trajectory (X-Y)", command=self.plot_trajectory, state=tk.DISABLED)
        self.plot_traj_btn.pack(fill=tk.X, pady=4, padx=10)
        
        self.plot_forces_btn = ttk.Button(plot_frame, text="Plot Tire Forces (Longitudinal/Lateral)", command=self.plot_forces, state=tk.DISABLED)
        self.plot_forces_btn.pack(fill=tk.X, pady=4, padx=10)

        self.plot_vert_forces_btn = ttk.Button(plot_frame, text="Plot Vertical Tire Forces (Fz)", command=self.plot_vertical_forces, state=tk.DISABLED)
        self.plot_vert_forces_btn.pack(fill=tk.X, pady=4, padx=10)
        
        self.plot_slips_btn = ttk.Button(plot_frame, text="Plot Tire Slips (Angle & Ratio)", command=self.plot_slips, state=tk.DISABLED)
        self.plot_slips_btn.pack(fill=tk.X, pady=4, padx=10)
        
        self.plot_speeds_btn = ttk.Button(plot_frame, text="Plot Wheel Spin Speeds & Torques", command=self.plot_speeds, state=tk.DISABLED)
        self.plot_speeds_btn.pack(fill=tk.X, pady=4, padx=10)

        self.plot_susp_btn = ttk.Button(plot_frame, text="Plot Suspension Deflections & Velocities", command=self.plot_suspension, state=tk.DISABLED)
        self.plot_susp_btn.pack(fill=tk.X, pady=4, padx=10)

        self.plot_ctrl_btn = ttk.Button(plot_frame, text="Plot Driver Control Efforts", command=self.plot_controls, state=tk.DISABLED)
        self.plot_ctrl_btn.pack(fill=tk.X, pady=4, padx=10)

    def start_simulation(self):
        self.run_sim_btn.config(state=tk.DISABLED)
        self.sim_status_lbl.config(text="Status: Simulation running...")
        
        t = threading.Thread(target=self.run_simulation_thread)
        t.daemon = True
        t.start()
        
    def run_simulation_thread(self):
        import subprocess
        exe_name = "demo_VEH_FMI2_WheeledVehicle_lanechange.exe"
        exe_path = os.path.join(SCRIPT_DIR, "build", "src", "demo_VEH_FMI2_WheeledVehicle_lanechange", exe_name)
        if not os.path.exists(exe_path):
            exe_path = os.path.join(SCRIPT_DIR, "build", "bin", exe_name)
            
        if not os.path.exists(exe_path):
            self.root.after(0, lambda: self.sim_status_lbl.config(text="Status: Executable not found! Run build_fmus.bat first."))
            self.root.after(0, lambda: self.run_sim_btn.config(state=tk.NORMAL))
            return
            
        cmd = [exe_path]
        
        # Determine visibility for vehicle and driver FMUs
        veh_vis = "1" if self.vehicle_vis_val.get() else "0"
        drv_vis = "1" if self.driver_vis_val.get() else "0"
        cmd.extend(["--vehicle_visible", veh_vis])
        cmd.extend(["--driver_visible", drv_vis])
            
        csv_path = os.path.join(SCRIPT_DIR, "build", "generated", "lane_change_trajectory.csv")
        cmd.extend(["--output", csv_path])
        
        t_end = safe_float(self.t_end_var.get(), 13.0)
        cmd.extend(["--tend", str(t_end)])
        
        # Pass generated path, CRG terrain, and speed profile files to the simulation
        path_file = os.path.join(SCRIPT_DIR, "build", "generated", "default_lane_change_path.txt")
        crg_file = os.path.join(SCRIPT_DIR, "build", "generated", "default_road.crg")
        speed_profile_file = os.path.join(SCRIPT_DIR, "build", "generated", "speed_profile.txt")
        cmd.extend(["--path_file", path_file])
        cmd.extend(["--terrain_crg_file", crg_file])
        cmd.extend(["--speed_profile_file", speed_profile_file])
        
        diffuse_tex = "" if self.tex_diffuse_var.get() == "None" else self.tex_diffuse_var.get()
        normal_tex = "" if self.tex_normal_var.get() == "None" else self.tex_normal_var.get()
        show_lines = "1" if self.show_lines_val.get() else "0"
        simplify_crg = "1" if self.simplify_mesh_val.get() else "0"
        
        terrain_type = "2" if self.terrain_type_box.get() == "OpenCRG" else "1"
        cmd.extend(["--terrain", terrain_type])
        
        if diffuse_tex:
            cmd.extend(["--terrain_diffuse_texture", diffuse_tex])
        if normal_tex:
            cmd.extend(["--terrain_normal_texture", normal_tex])
            
        cmd.extend(["--terrain_show_visual_lines", show_lines])
        cmd.extend(["--simplify_crg", simplify_crg])
        
        cmd.extend(["--steering_type", "1" if self.steer_type_box.get() == "Stanley" else "0"])
        cmd.extend(["--look_ahead_dist", str(safe_float(self.look_ahead_var.get(), 3.6))])
        cmd.extend(["--Kp_steering", str(safe_float(self.kp_steer_var.get(), 2.39))])
        cmd.extend(["--Ki_steering", str(safe_float(self.ki_steer_var.get(), 0.0))])
        cmd.extend(["--Kd_steering", str(safe_float(self.kd_steer_var.get(), 0.0))])
        cmd.extend(["--stanley_dead_zone", str(safe_float(self.dead_zone_var.get(), 0.01))])
        cmd.extend(["--max_torque", "350.0"])
        cmd.extend(["--init_vel", str(safe_float(self.speed_init_var.get(), 60.0) / 3.6)])
        
        print(f"\nRunning simulation: {' '.join(cmd)}\n")
        
        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=SCRIPT_DIR)
            for line in process.stdout:
                sys.stdout.write(line)
            process.wait()
            
            if process.returncode == 0:
                self.root.after(0, lambda: self.sim_status_lbl.config(text="Status: Simulation completed successfully!"))
                self.root.after(0, self.enable_telemetry_buttons)
            else:
                self.root.after(0, lambda: self.sim_status_lbl.config(text=f"Status: Simulation failed with code {process.returncode}"))
        except Exception as e:
            self.root.after(0, lambda: self.sim_status_lbl.config(text=f"Status: Error running simulation: {e}"))
            
        self.root.after(0, lambda: self.run_sim_btn.config(state=tk.NORMAL))
        
    def enable_telemetry_buttons(self):
        self.plot_traj_btn.config(state=tk.NORMAL)
        self.plot_forces_btn.config(state=tk.NORMAL)
        self.plot_vert_forces_btn.config(state=tk.NORMAL)
        self.plot_slips_btn.config(state=tk.NORMAL)
        self.plot_speeds_btn.config(state=tk.NORMAL)
        self.plot_susp_btn.config(state=tk.NORMAL)
        self.plot_ctrl_btn.config(state=tk.NORMAL)

    def load_telemetry_csv(self):
        csv_path = os.path.join(SCRIPT_DIR, "build", "generated", "lane_change_trajectory.csv")
        if not os.path.exists(csv_path):
            messagebox.showerror("Error", f"Telemetry file not found at: {csv_path}")
            return None
        try:
            data = np.loadtxt(csv_path)
            return data
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load telemetry CSV: {e}")
            return None

    def plot_trajectory(self):
        data = self.load_telemetry_csv()
        if data is None: return
        import matplotlib.pyplot as plt
        
        t = data[:, 0]
        x = data[:, 1]
        y = data[:, 2]
        
        plt.figure("Vehicle Trajectory (X-Y)", figsize=(8, 6))
        plt.plot(x, y, "b-", linewidth=2, label="Vehicle Trajectory")
        
        # Load and plot reference path if it exists
        ref_path_file = os.path.join(SCRIPT_DIR, "build", "generated", "default_lane_change_path.txt")
        if os.path.exists(ref_path_file):
            try:
                ref_pts = []
                with open(ref_path_file, "r") as f:
                    for line in f:
                        parts = line.strip().split()
                        if len(parts) >= 2:
                            ref_pts.append((float(parts[0]), float(parts[1])))
                if ref_pts:
                    ref_x = [p[0] for p in ref_pts]
                    ref_y = [p[1] for p in ref_pts]
                    plt.plot(ref_x, ref_y, "r--", linewidth=1.5, label="Reference Path")
            except Exception as e:
                print(f"Warning: Failed to load reference path for plotting: {e}")
                
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Vehicle Top-Down Trajectory (Proportional 1:1)")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.show()
        
    def plot_forces(self):
        data = self.load_telemetry_csv()
        if data is None: return
        import matplotlib.pyplot as plt
        
        t = data[:, 0]
        fx_fl, fy_fl = data[:, 11], data[:, 12]
        fx_fr, fy_fr = data[:, 14], data[:, 15]
        fx_rl, fy_rl = data[:, 17], data[:, 18]
        fx_rr, fy_rr = data[:, 20], data[:, 21]
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        
        ax1.plot(t, fx_fl, "r-", label="FL")
        ax1.plot(t, fx_fr, "g-", label="FR")
        ax1.plot(t, fx_rl, "b-", label="RL")
        ax1.plot(t, fx_rr, "k-", label="RR")
        ax1.set_ylabel("Longitudinal Force Fx (N)")
        ax1.set_title("Tire Contact Forces Telemetry")
        ax1.grid(True)
        ax1.legend()
        
        ax2.plot(t, fy_fl, "r-", label="FL")
        ax2.plot(t, fy_fr, "g-", label="FR")
        ax2.plot(t, fy_rl, "b-", label="RL")
        ax2.plot(t, fy_rr, "k-", label="RR")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Lateral Force Fy (N)")
        ax2.grid(True)
        ax2.legend()
        
        plt.tight_layout()
        plt.show()
        
    def plot_slips(self):
        data = self.load_telemetry_csv()
        if data is None: return
        import matplotlib.pyplot as plt
        
        t = data[:, 0]
        
        sa_fl, sa_fr = data[:, 31], data[:, 32]
        sa_rl, sa_rr = data[:, 33], data[:, 34]
        
        sr_fl, sr_fr = data[:, 35], data[:, 36]
        sr_rl, sr_rr = data[:, 37], data[:, 38]
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        
        ax1.plot(t, np.degrees(sa_fl), "r-", label="FL")
        ax1.plot(t, np.degrees(sa_fr), "g-", label="FR")
        ax1.plot(t, np.degrees(sa_rl), "b-", label="RL")
        ax1.plot(t, np.degrees(sa_rr), "k-", label="RR")
        ax1.set_ylabel("Slip Angle (degrees)")
        ax1.set_title("Tire Slip Telemetry")
        ax1.grid(True)
        ax1.legend()
        
        ax2.plot(t, sr_fl * 100.0, "r-", label="FL")
        ax2.plot(t, sr_fr * 100.0, "g-", label="FR")
        ax2.plot(t, sr_rl * 100.0, "b-", label="RL")
        ax2.plot(t, sr_rr * 100.0, "k-", label="RR")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Slip Ratio (%)")
        ax2.grid(True)
        ax2.legend()
        
        plt.tight_layout()
        plt.show()
        
    def plot_speeds(self):
        data = self.load_telemetry_csv()
        if data is None: return
        import matplotlib.pyplot as plt
        
        t = data[:, 0]
        
        trq_fl, trq_fr = data[:, 23], data[:, 24]
        trq_rl, trq_rr = data[:, 25], data[:, 26]
        
        w_fl, w_fr = data[:, 27], data[:, 28]
        w_rl, w_rr = data[:, 29], data[:, 30]
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        
        ax1.plot(t, trq_fl, "r-", label="FL")
        ax1.plot(t, trq_fr, "g-", label="FR")
        ax1.plot(t, trq_rl, "b-", label="RL")
        ax1.plot(t, trq_rr, "k-", label="RR")
        ax1.set_ylabel("Wheel Torque (Nm)")
        ax1.set_title("Wheel Speed & Drive Torque Telemetry")
        ax1.grid(True)
        ax1.legend()
        
        ax2.plot(t, w_fl, "r-", label="FL")
        ax2.plot(t, w_fr, "g-", label="FR")
        ax2.plot(t, w_rl, "b-", label="RL")
        ax2.plot(t, w_rr, "k-", label="RR")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Angular Speed (rad/s)")
        ax2.grid(True)
        ax2.legend()
        
        plt.tight_layout()
        plt.show()

    def plot_vertical_forces(self):
        data = self.load_telemetry_csv()
        if data is None: return
        import matplotlib.pyplot as plt
        
        t = data[:, 0]
        fz_fl = data[:, 13]
        fz_fr = data[:, 16]
        fz_rl = data[:, 19]
        fz_rr = data[:, 22]
        
        plt.figure("Vertical Tire Forces (Fz)", figsize=(9, 6))
        plt.plot(t, fz_fl, "r-", label="FL")
        plt.plot(t, fz_fr, "g-", label="FR")
        plt.plot(t, fz_rl, "b-", label="RL")
        plt.plot(t, fz_rr, "k-", label="RR")
        plt.xlabel("Time (s)")
        plt.ylabel("Vertical Force Fz (N)")
        plt.title("Vertical Tire Contact Forces Telemetry")
        plt.grid(True)
        plt.legend()
        plt.show()

    def plot_suspension(self):
        data = self.load_telemetry_csv()
        if data is None: return
        import matplotlib.pyplot as plt
        
        t = data[:, 0]
        if data.shape[1] >= 43:
            trav_fl, trav_fr = data[:, 39], data[:, 40]
            trav_rl, trav_rr = data[:, 41], data[:, 42]
        else:
            print("Warning: Older CSV file without suspension travel data.")
            trav_fl = trav_fr = trav_rl = trav_rr = np.zeros(len(t))
            
        vel_fl, vel_fr = data[:, 7], data[:, 8]
        vel_rl, vel_rr = data[:, 9], data[:, 10]
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        
        ax1.plot(t, trav_fl, "r-", label="FL")
        ax1.plot(t, trav_fr, "g-", label="FR")
        ax1.plot(t, trav_rl, "b-", label="RL")
        ax1.plot(t, trav_rr, "k-", label="RR")
        ax1.set_ylabel("Travel (m)")
        ax1.set_title("Suspension Deflection & Velocity Telemetry")
        ax1.grid(True)
        ax1.legend()
        
        ax2.plot(t, vel_fl, "r-", label="FL")
        ax2.plot(t, vel_fr, "g-", label="FR")
        ax2.plot(t, vel_rl, "b-", label="RL")
        ax2.plot(t, vel_rr, "k-", label="RR")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Velocity (m/s)")
        ax2.grid(True)
        ax2.legend()
        
        plt.tight_layout()
        plt.show()

    def plot_controls(self):
        data = self.load_telemetry_csv()
        if data is None: return
        import matplotlib.pyplot as plt
        
        t = data[:, 0]
        if data.shape[1] >= 46:
            steering = data[:, 43]
            throttle = data[:, 44]
            braking = data[:, 45]
        else:
            print("Warning: Older CSV file without driver control effort data.")
            steering = throttle = braking = np.zeros(len(t))
            
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        
        ax1.plot(t, steering, "b-", linewidth=2, label="Steering Command")
        ax1.set_ylabel("Steering (-1 to 1)")
        ax1.set_title("Driver Control Effort Telemetry")
        ax1.grid(True)
        ax1.legend()
        
        ax2.plot(t, throttle, "g-", linewidth=2, label="Throttle")
        ax2.plot(t, braking, "r-", linewidth=2, label="Braking")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Pedals (0 to 1)")
        ax2.grid(True)
        ax2.legend()
        
        plt.tight_layout()
        plt.show()

    def start_3d_viewer(self):
        crg_path = os.path.join(SCRIPT_DIR, "build", "generated", "default_road.crg")
        if not os.path.exists(crg_path):
            messagebox.showerror("Error", f"No generated OpenCRG file found at: {crg_path}. Run road generation first.")
            return
        
        t = threading.Thread(target=self.run_3d_viewer_thread, args=(crg_path,))
        t.daemon = True
        t.start()
        
    def run_3d_viewer_thread(self, filepath):
        try:
            print(f"\nParsing OpenCRG and starting 3D CAD viewer for: {filepath}...\n")
            headers = {}
            data_start_idx = 0
            with open(filepath, "r") as f:
                for idx, line in enumerate(f):
                    line_str = line.strip()
                    if line_str == "$$$$":
                        data_start_idx = idx + 1
                        break
                    if "=" in line_str:
                        parts = line_str.split("=")
                        headers[parts[0].strip()] = parts[1].strip()
            
            with open(filepath, "r") as f:
                content = f.read()
            
            data_start_pos = content.find("$$$$")
            data_part = content[data_start_pos + 4:] if data_start_pos != -1 else content
            
            import re
            # Preprocess packed signs by adding space separators (avoiding exponents like e-05)
            data_part = re.sub(r'(?<![eE])-', ' -', data_part)
            data_part = re.sub(r'(?<![eE])\+', ' +', data_part)
            
            all_vals = np.fromstring(data_part, dtype=float, sep=' ')
            
            u_start = float(headers.get("reference_line_start_u", 0.0))
            u_end = float(headers.get("reference_line_end_u", 100.0))
            u_inc = float(headers.get("reference_line_increment", 0.1))
            
            v_right = float(headers.get("long_section_v_right", -4.0))
            v_left = float(headers.get("long_section_v_left", 4.0))
            v_inc = float(headers.get("long_section_v_increment", 0.1))
            
            ref_x0 = float(headers.get("reference_line_start_x", 0.0))
            ref_y0 = float(headers.get("reference_line_start_y", 0.0))
            ref_phi0 = float(headers.get("reference_line_start_phi", 0.0))
            
            u_grid = np.arange(u_start, u_end + u_inc/2, u_inc)
            v_grid = np.arange(v_right, v_left + v_inc/2, v_inc)
            Nu = len(u_grid)
            Nv = len(v_grid)
            
            record_size = 3 + Nv
            num_records = min(Nu, len(all_vals) // record_size)
            
            phi = np.zeros(num_records)
            banking = np.zeros(num_records)
            elevations = np.zeros((num_records, Nv))
            
            for i in range(num_records):
                offset = i * record_size
                phi[i] = all_vals[offset]
                banking[i] = all_vals[offset + 2]
                elevations[i, :] = all_vals[offset + 3 : offset + 3 + Nv]
                
            x_ref = np.zeros(num_records)
            y_ref = np.zeros(num_records)
            x_ref[0] = ref_x0
            y_ref[0] = ref_y0
            for i in range(1, num_records):
                du = u_grid[i] - u_grid[i-1]
                x_ref[i] = x_ref[i-1] + du * np.cos(phi[i-1])
                y_ref[i] = y_ref[i-1] + du * np.sin(phi[i-1])
                
            X = np.zeros((num_records, Nv))
            Y = np.zeros((num_records, Nv))
            for j in range(Nv):
                v = v_grid[j]
                X[:, j] = x_ref - v * np.cos(banking) * np.sin(phi)
                Y[:, j] = y_ref + v * np.cos(banking) * np.cos(phi)
            Z = elevations
            
            import pyvista as pv
            points = np.empty((num_records, Nv, 3))
            points[:, :, 0] = X
            points[:, :, 1] = Y
            points[:, :, 2] = Z
            flat_pts = points.reshape(-1, 3)
            
            grid = pv.StructuredGrid()
            grid.dimensions = (Nv, num_records, 1)
            grid.points = flat_pts
            grid.point_data["Elevation (m)"] = Z.flatten()
            
            # Apply texture coordinates analytically to warp and repeat cleanly along the curves
            road_length = u_grid[-1] - u_grid[0] if len(u_grid) > 0 else 100.0
            road_width = v_grid[-1] - v_grid[0] if len(v_grid) > 0 else 8.0
            v_repeats = 0.5 * road_length / (road_width if road_width > 0.0 else 8.0)
            
            tcoords = np.zeros((num_records * Nv, 2))
            for i in range(num_records):
                for j in range(Nv):
                    idx = i * Nv + j
                    tcoords[idx, 0] = j / (Nv - 1) if Nv > 1 else 0.0
                    tcoords[idx, 1] = (i / (num_records - 1)) * v_repeats if num_records > 1 else 0.0
            grid.active_texture_coordinates = tcoords
            
            plotter = pv.Plotter(title="Interactive CAD 3D Road Viewer")
            plotter.set_background("#0f0f11")
            
            # Load diffuse texture if configured
            tex_path = resolve_texture_path(self.tex_diffuse_var.get())
            texture = None
            if tex_path and os.path.exists(tex_path):
                try:
                    texture = pv.read_texture(tex_path)
                except Exception as e:
                    print(f"Failed to read texture: {e}")
                    
            if texture:
                mesh_actor = plotter.add_mesh(grid, texture=texture, show_edges=False, smooth_shading=True)
            else:
                mesh_actor = plotter.add_mesh(grid, scalars="Elevation (m)", cmap="viridis", show_edges=False, smooth_shading=True)
            
            def update_z_scale(value):
                scaled_pts = flat_pts.copy()
                scaled_pts[:, 2] = flat_pts[:, 2] * value
                grid.points = scaled_pts
                plotter.render()
                
            plotter.add_slider_widget(update_z_scale, [1.0, 10.0], value=1.0, title="Z Scale Factor", pointa=(0.05, 0.08), pointb=(0.35, 0.08), color="#6366f1")
            plotter.show_grid(color="#27272a")
            plotter.show()
        except Exception as e:
            print(f"Error launching 3d viewer: {e}")

class ToolTip:
    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tip_window = None
        self.widget.bind("<Enter>", self.show_tip)
        self.widget.bind("<Leave>", self.hide_tip)

    def show_tip(self, event=None):
        if self.tip_window or not self.text:
            return
        x, y, cx, cy = self.widget.bbox("insert")
        x = x + self.widget.winfo_rootx() + 25
        y = y + self.widget.winfo_rooty() + 20
        self.tip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(1)
        tw.wm_geometry(f"+{x}+{y}")
        label = tk.Label(tw, text=self.text, justify=tk.LEFT,
                         background="#ffffe0", foreground="#000000", relief=tk.SOLID, borderwidth=1,
                         font=("Segoe UI", 8, "normal"))
        label.pack(ipadx=4, ipady=2)

    def hide_tip(self, event=None):
        tw = self.tip_window
        self.tip_window = None
        if tw:
            tw.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = RoadGeneratorGUI(root)
    root.mainloop()
