import unittest
import tkinter as tk
import os
import tempfile
import json
import numpy as np
import re
import shutil
from unittest.mock import MagicMock, patch

# Ensure script dir and road_generator dir are in path
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "src", "road_generator"))

import road_generator_gui as gui

class TestRoadGeneratorGUI(unittest.TestCase):
    def setUp(self):
        # Mock Tkinter messageboxes to prevent tests from hanging on modal dialogs
        self.patcher_info = patch("tkinter.messagebox.showinfo")
        self.patcher_error = patch("tkinter.messagebox.showerror")
        self.mock_info = self.patcher_info.start()
        self.mock_error = self.patcher_error.start()

    def tearDown(self):
        self.patcher_info.stop()
        self.patcher_error.stop()

    def test_safe_float(self):
        """Test safe_float parsing under normal and abnormal strings."""
        self.assertEqual(gui.safe_float("123.45"), 123.45)
        self.assertEqual(gui.safe_float("-45.6e-2"), -0.456)
        self.assertEqual(gui.safe_float("abc", fallback=5.0), 5.0)
        self.assertEqual(gui.safe_float("  12  "), 12.0)
        self.assertEqual(gui.safe_float("12-34", fallback=1.0), 1.0) # clean logic joins digits and dot/sign/exponents: '12-34' -> 12-34 -> float fails -> returns fallback

    def test_redirect_text(self):
        """Test RedirectText successfully writes to a queue."""
        import queue
        q = queue.Queue()
        redirector = gui.RedirectText(q)
        redirector.write("Hello World\n")
        self.assertEqual(q.get(), "Hello World\n")

    def test_get_preview_points(self):
        """Test mathematical path preview points calculation for various maneuvers."""
        # 1. Straight Line
        pts = gui.get_preview_points(
            speed_init=60.0, speed_target=60.0, t_speed_start=0.0, t_speed_dur=1.0,
            t_end=10.0, maneuver="Straight Line", t_start=2.0, t_dur=5.0, width=5.0,
            dwell=2.0, radius=50.0, direction="Left", lead_in=20.0, start_margin=10.0, end_margin=10.0
        )
        self.assertEqual(len(pts), 111) # 101 base points + 5 start margin + 5 end margin points
        for p in pts:
            self.assertEqual(len(p), 2) # x, y (preview points are 2D coordinates)

        # 2. Single Lane Change
        pts = gui.get_preview_points(
            speed_init=60.0, speed_target=60.0, t_speed_start=0.0, t_speed_dur=1.0,
            t_end=10.0, maneuver="Single Lane Change", t_start=2.0, t_dur=5.0, width=5.0,
            dwell=2.0, radius=50.0, direction="Left", lead_in=20.0, start_margin=10.0, end_margin=10.0
        )
        self.assertEqual(len(pts), 111)

        # 3. Circular Path
        pts = gui.get_preview_points(
            speed_init=60.0, speed_target=60.0, t_speed_start=0.0, t_speed_dur=1.0,
            t_end=10.0, maneuver="Circular Path", t_start=2.0, t_dur=5.0, width=5.0,
            dwell=2.0, radius=50.0, direction="Left", lead_in=20.0, start_margin=10.0, end_margin=10.0
        )
        self.assertEqual(len(pts), 111)

    def test_opencrg_parser_regex(self):
        """Verify the fast parser preprocesses and extracts packed negative decimals correctly."""
        test_line = "0.0000310-0.0002928-0.0005930-0.0008711-0.0011270-0.0013507-0.0015185"
        
        # Apply space separator formatting
        data_part = re.sub(r'(?<![eE])-', ' -', test_line)
        data_part = re.sub(r'(?<![eE])\+', ' +', data_part)
        
        # Parse using numpy fromstring
        vals = np.fromstring(data_part, dtype=float, sep=' ')
        
        self.assertEqual(len(vals), 7)
        self.assertEqual(vals[0], 0.0000310)
        self.assertEqual(vals[1], -0.0002928)
        self.assertEqual(vals[6], -0.0015185)

    def test_settings_load_save(self):
        """Test settings load and save operations via mock temp files."""
        temp_dir = tempfile.mkdtemp()
        temp_settings = os.path.join(temp_dir, "test_settings.json")
        
        try:
            with patch("road_generator_gui.SETTINGS_FILE", temp_settings):
                # Setup GUI environment
                root = tk.Tk()
                root.withdraw() # Headless Tk
                
                app = gui.RoadGeneratorGUI(root)
                # Modify target speed field
                app.update_entry(app.speed_target_var, "85.5")
                app.maneuver_box.set("Circular Path")
                app.save_settings()
                
                # Verify settings file exists and matches
                self.assertTrue(os.path.exists(temp_settings))
                with open(temp_settings, "r") as f:
                    data = json.load(f)
                self.assertEqual(data.get("target_speed_kph"), 85.5)
                self.assertEqual(data.get("maneuver_type"), "Circular Path")
                
                # Check load
                loaded = app.load_settings()
                self.assertEqual(loaded.get("target_speed_kph"), 85.5)
                
                root.destroy()
        finally:
            if os.path.exists(temp_dir):
                shutil.rmtree(temp_dir)

    def test_presets_application(self):
        """Test application of presets updates input fields correctly."""
        root = tk.Tk()
        root.withdraw()
        
        app = gui.RoadGeneratorGUI(root)
        
        # Test preset Single Lane Change
        app.maneuver_box.set("Single Lane Change")
        app.reset_maneuver_defaults()
        
        self.assertEqual(float(app.t_start_var.get()), 2.0)
        self.assertEqual(float(app.t_dur_var.get()), 5.0)
        self.assertEqual(float(app.width_var.get()), 5.0)
        
        # Test preset Slalom
        app.maneuver_box.set("Slalom")
        app.reset_maneuver_defaults()
        self.assertEqual(float(app.slalom_amp_var.get()), 1.5)
        self.assertEqual(float(app.slalom_period_var.get()), 36.0)
        
        root.destroy()

    def test_tooltips_creation(self):
        """Verify tooltips bind successfully to widgets."""
        root = tk.Tk()
        root.withdraw()
        
        btn = tk.Button(root, text="Test Button")
        btn.pack()
        
        tooltip = gui.ToolTip(btn, text="This is a test hover text")
        self.assertIsNotNone(tooltip)
        
        # Call hover bindings directly to cover widget state transitions
        tooltip.show_tip(None)
        self.assertIsNotNone(tooltip.tip_window)
        tooltip.hide_tip(None)
        self.assertIsNone(tooltip.tip_window)
        
        root.destroy()

    @patch("road_generator_gui.generate_road_profile")
    def test_gui_generation_fields_read(self, mock_generate):
        """Verify that all referenced widget entry/combobox variables exist and are read without AttributeErrors when running road generation."""
        root = tk.Tk()
        root.withdraw()
        
        app = gui.RoadGeneratorGUI(root)
        
        # Manually run the generation method to check if any self.*_var read calls fail
        try:
            app.run_generator()
            # Assert that it reached the mocked API call successfully
            mock_generate.assert_called_once()
        except Exception as e:
            self.fail(f"run_generator raised exception: {e}")
        finally:
            root.destroy()

    def test_actual_road_generation_low_res(self):
        """Test actual road generator C++ execution with low resolution grid on the fly."""
        temp_dir = tempfile.mkdtemp()
        try:
            # Create build directory under temp_dir and copy compiled generate_road.exe
            os.path.join(temp_dir, "build")
            os.makedirs(os.path.join(temp_dir, "build"), exist_ok=True)
            src_exe = os.path.join(gui.SCRIPT_DIR, "build", "generate_road.exe")
            dest_exe = os.path.join(temp_dir, "build", "generate_road.exe")
            shutil.copy2(src_exe, dest_exe)

            import road_generator as rg
            
            # base_dir points to the temp folder; the generator writes to base_dir/build/generated/
            rg.generate_road_profile(
                target_speed_kph=40.0,
                t_start=2.0,
                t_duration=3.0,
                t_end=6.0,
                width=4.0,
                start_length_margin=5.0,
                end_length_margin=5.0,
                mesh_resolution=1.5,      # Very low resolution grid (large grid spacing)
                v_width=6.0,
                iso_class="A",
                generate_obj=False,       # Skip OBJ mesh to run faster
                base_dir=temp_dir,
                maneuver_type="Straight Line",
                dwell_time=0.0,
                radius=0.0,
                direction="Left",
                lead_in_length=0.0,
                slope_type="None",
                slope_value=0.0,
                slope_start=0.0,
                slope_duration=0.0,
                banking_type="None",
                banking_value=0.0,
                banking_start=0.0,
                banking_duration=0.0,
                curvature_filter_window_m=0.0,
                mu_value=0.85,
                speed_profile_time_start=0.0,
                speed_profile_duration=0.0,
                speed_profile_initial_speed=40.0,
                speed_profile_target_speed=40.0,
                steering_type=1,
                look_ahead_dist=3.0,
                Kp_steering=2.0,
                Ki_steering=0.0,
                Kd_steering=0.0,
                stanley_dead_zone=0.0,
                max_wheel_turn_angle=25.0,
                Kp_speed=1.0,
                Ki_speed=0.0,
                Kd_speed=0.0,
                roughness_Nf=4,           # Only 4 frequency divisions (fast wave sum!)
                roughness_Ntheta=4,       # Only 4 angles (total 16 waves instead of 16384!)
            )
            
            # Verify outputs were generated in the temp directory's build/generated path
            gen_path = os.path.join(temp_dir, "build", "generated")
            self.assertTrue(os.path.exists(os.path.join(gen_path, "default_road.crg")))
            self.assertTrue(os.path.exists(os.path.join(gen_path, "simulation_parameters.m")))
            self.assertTrue(os.path.exists(os.path.join(gen_path, "speed_profile.txt")))
        finally:
            if os.path.exists(temp_dir):
                shutil.rmtree(temp_dir)

if __name__ == "__main__":
    unittest.main()
