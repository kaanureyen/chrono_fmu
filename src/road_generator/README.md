# Lane-Change Path and OpenCRG Terrain Generator

This directory contains the tools and generated resources for setting up the co-simulation lane change maneuver. The generator (`road_generator.py`) coordinates the creation of both:
1. **Driver Path (`default_lane_change_path.txt`)**: A Bezier spline path followed by the Driver FMU's steering controller.
2. **Vehicle Terrain (`default_road.crg`)**: An OpenCRG rough road surface followed by the Vehicle FMU's physics solver.

By sharing the underlying geometry equations, the generator ensures that the driver's target trajectory and the terrain's physical centerline are perfectly aligned.

---

## 1. Lane-Change Path Geometry
The lane change maneuver is modeled using a 7th-order smooth step function $S(\tau)$, which provides continuous first, second, and third derivatives to prevent step changes in target lateral acceleration and steering rate.

### Equations:
The trajectory coordinates are:
* $x(t) = v t$
* $y(t) = \begin{cases} 0 & t < t_{start} \\ w_L S(\tau) & t_{start} \le t \le t_{start} + t_{dur} \\ w_L & t > t_{start} + t_{dur} \end{cases}$

where:
* $v = 22.2222 \text{ m/s}$ (80 kph target velocity)
* $t_{start} = 2.0 \text{ s}$ (maneuver start time)
* $t_{dur} = 5.0 \text{ s}$ (maneuver duration)
* $w_L = 5.0 \text{ m}$ (lane change lateral width)
* $\tau = \frac{t - t_{start}}{t_{dur}} \in [0, 1]$
* $S(\tau) = 35 \tau^4 - 84 \tau^5 + 70 \tau^6 - 20 \tau^7$

---

## 2. Curved OpenCRG Reference Line
OpenCRG represents road geometry by defining a centerline reference line and cross-sections orthogonal to it. To make the physical road follow the curved lane change path, the generator performs the following steps:

1. **Arc-Length Integration**:
   We numerically integrate the arc length $u(t)$ along the trajectory:
   $$u(t) = \int_0^t \sqrt{\dot{x}(t')^2 + \dot{y}(t')^2} dt'$$
2. **Heading Angle Calculation**:
   At each longitudinal station $u_i$ along the road, we find the corresponding time $t_i$ and evaluate the heading angle $\phi(u_i)$:
   $$\phi(u_i) = \text{atan2}(\dot{y}(t_i), \dot{x}(t_i))$$
   This heading angle is written into the `D:reference line phi,rad` data channel of the OpenCRG file.
3. **Cross-Section Coordinates**:
   For any grid point $(u_i, v_j)$, the global coordinates $(x_g, y_g)$ are computed normal to the centerline:
   $$x_g = x_{ref}(u_i) - v_j \sin(\phi(u_i))$$
   $$y_g = y_{ref}(u_i) + v_j \cos(\phi(u_i))$$
   where $v_j$ is the lateral coordinate from $-6.0$ to $+6.0$ meters.

---

## 3. Surface Roughness & Nyquist Discretization

### ISO 8608 Class C Roughness
The surface elevation profile is modeled as a 2D random process using the superposition of $256 \times 16 = 4096$ harmonic wave components:
$$z(x_g, y_g) = \sum_{m=1}^{256} \sum_{n=1}^{16} A_m \cos\left(2\pi f_m (x_g \cos\theta_n + y_g \sin\theta_n) + \theta_{mn}\right)$$

* **PSD Parameters**: $G_d(n_0) = 256 \times 10^{-6} \text{ m}^3$ at reference frequency $n_0 = 0.1 \text{ cycles/m}$.
* **Wavenumber Exponent**: $w = 2.0$.
* **Frequency Range**: $f_{min} = 0.01 \text{ cycles/m}$ to $f_{max} = 2.0 \text{ cycles/m}$.

### Nyquist-Shannon Sampling Discretization
To prevent spatial aliasing when modeling frequencies up to $f_{max} = 2.0 \text{ cycles/m}$, the grid spacing must satisfy the Nyquist sampling theorem:
$$\Delta \le \frac{1}{2 f_{max}} = 0.25 \text{ m}$$

* **Longitudinal spacing ($du$)**: Set to **$0.2 \text{ m}$** (sampling rate $5.0 \text{ samples/m} > 4.0 \text{ samples/m}$).
* **Lateral spacing ($dv$)**: Set to **$0.2 \text{ m}$** (sampling rate $5.0 \text{ samples/m} > 4.0 \text{ samples/m}$).
* **Grid Resolution**: $2001$ stations by $61$ lateral tracks, yielding a total of **$122,061$ height points**.

---

## 4. Usage Instructions

To regenerate the path and road files and copy them to the Driver and Vehicle FMU resources:

1. Run the Python script:
   ```cmd
   python road_generator.py
   ```
2. Recompile and package the FMUs by running the build script in the parent directory:
   ```cmd
   ..\build_fmus.bat
   ```
   This will clean the build directory, compile the FMU shared libraries, inject the updated description attributes, and bundle the new `default_road.crg` and `default_lane_change_path.txt` files inside the final `.fmu` zip packages.
