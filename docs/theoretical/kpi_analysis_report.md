# Vehicle Dynamics KPI Analysis Report
## Torque Vectoring & Active Suspension Studies

This report provides a systematic analysis of Key Performance Indicators (KPIs) for evaluating vehicle comfort, performance/handling, tire wear, safety, and energy efficiency. It also analyzes the gap between currently available FMU outputs and the variables required to calculate these KPIs.

---

## 1. Key Performance Indicators (KPIs) and Mathematical Formulations

### 1.1. Comfort (Ride Quality)
Comfort is dominated by how well the suspension isolates the cabin from high-frequency road inputs.

#### A. Chassis Vertical Acceleration RMS
The Root-Mean-Square (RMS) of vertical acceleration is the standard metric (ISO 2631) for evaluating whole-body vibration.
\[
\text{RMS}(a_z) = \sqrt{\frac{1}{T} \int_{0}^{T} a_z(t)^2 \, dt}
\]
Where:
- \(a_z(t)\) is the vertical acceleration of the chassis center of mass (`ref_frame.pos_dtdt.z`).

#### B. Pitch and Roll Rate RMS
Minimizing rotational velocities is critical to preventing passenger motion sickness and vehicle instability.
\[
\text{RMS}(\omega_x) = \sqrt{\frac{1}{T} \int_{0}^{T} \omega_x(t)^2 \, dt}, \quad \text{RMS}(\omega_y) = \sqrt{\frac{1}{T} \int_{0}^{T} \omega_y(t)^2 \, dt}
\]
Where:
- \(\omega_x, \omega_y\) are the roll and pitch angular velocities of the chassis (derived from `ref_frame.rot_dt`).

#### C. Suspension Workspace Utilization (Travel)
Ensuring the suspension does not hit bump stops. Hitting bump stops causes extremely high transient force spikes, ruining comfort.
\[
s_{\text{travel}, i}(t) = z_{\text{wheel}, i}(t) - z_{\text{chassis}, i}(t)
\]
\[
\text{Workspace Utilization Ratio} = \frac{\max(|s_{\text{travel}, i}|)}{s_{\text{max}}}
\]
Where:
- \(s_{\text{max}}\) is the maximum design stroke of the suspension.

---

### 1.2. Performance & Handling (Lateral/Longitudinal Control)
Handling metrics evaluate steering response, tracking deviation, and stability margins.

#### A. Yaw Rate Tracking Error
Evaluates the effectiveness of a torque vectoring controller.
\[
e_{\omega_z}(t) = \omega_z(t) - \omega_{z,\text{des}}(t)
\]
The desired yaw rate \(\omega_{z,\text{des}}\) is derived from the linear single-track (bicycle) model:
\[
\omega_{z,\text{des}} = \frac{v_x}{L + K_{us} v_x^2} \cdot \delta_f
\]
Where:
- \(L\) is the wheelbase.
- \(K_{us}\) is the understeer gradient.
- \(v_x\) is the forward velocity (`ref_frame.pos_dt.x`).
- \(\delta_f\) is the front wheel steer angle (derived from the output command `steering`).

#### B. Body Sideslip Angle (\(\beta\))
The angle between the longitudinal axis and the velocity vector. Maintaining a low sideslip angle ensures lateral stability.
\[
\beta = \arctan\left(\frac{v_y}{v_x}\right)
\]
Where:
- \(v_x, v_y\) are the chassis velocities (`ref_frame.pos_dt.x` and `ref_frame.pos_dt.y`).

#### C. Cross-Track Error (\(e_y\))
Measures the lateral deviation from the desired path.
\[
e_y = \min_{\mathbf{p}_{\text{path}}} \|\mathbf{p}_{\text{chassis}} - \mathbf{p}_{\text{path}}\|
\]
Where:
- \(\mathbf{p}_{\text{chassis}}\) is the vehicle position vector (`ref_frame.pos`).

---

### 1.3. Safety & Rollover Protection

#### A. Rollover Index (RI) / Dynamic Load Transfer
Evaluates lateral rollover danger during high-speed cornering.
\[
\text{RI} = \frac{|F_{z,\text{left}} - F_{z,\text{right}}|}{F_{z,\text{left}} + F_{z,\text{right}}}
\]
Where:
- \(F_{z,\text{left}} = F_{z,\text{FL}} + F_{z,\text{RL}}\)
- \(F_{z,\text{right}} = F_{z,\text{FR}} + F_{z,\text{RR}}\)
- A rollover index \(\text{RI} \to 1.0\) indicates wheel lift-off, meaning roll stability limits are reached.

---

### 1.4. Tire Wear & Dynamic Grip

#### A. Dynamic Normal Load Variation Coefficient (Road Holding)
Grip limit is proportional to tire vertical force. Minimizing the variation of \(F_z\) ensures consistent lateral and longitudinal grip.
\[
\text{COV}(F_z) = \frac{\sigma(F_z)}{\mu(F_z)}
\]
Where:
- \(\sigma(F_z)\) is the standard deviation of normal force.
- \(\mu(F_z)\) is the mean normal force.

#### B. Friction Energy Dissipation (Tire Wear Index)
Directly correlates to the rate of tread wear due to friction heating and mechanical abrasion.
\[
E_{\text{wear}, i} = \int_{0}^{T} \left( |F_{x, i} \cdot v_{\text{slip}, x, i}| + |F_{y, i} \cdot v_{\text{slip}, y, i}| \right) \, dt
\]
Where:
- \(F_x, F_y\) are the longitudinal and lateral tire contact forces.
- \(v_{\text{slip}, x}, v_{\text{slip}, y}\) are the sliding velocities of the tire contact patch.

---

### 1.5. Actuator Energy Consumption
Evaluates the efficiency cost of active control.

#### A. Active Suspension Power
\[
P_{\text{act}} = \sum_{i \in \{\text{FL, FR, RL, RR}\}} |F_{\text{act}, i} \cdot v_{\text{susp}, i}|
\]
Where:
- \(F_{\text{act}, i}\) is the active control force applied (`act_force_<ID>`).
- \(v_{\text{susp}, i}\) is the relative suspension stroke velocity.

#### B. Traction Power (Torque Vectoring)
\[
P_{\text{drive}} = \sum_{i \in \{\text{FL, FR, RL, RR}\}} |T_{\text{motor}, i} \cdot \omega_{\text{wheel}, i}|
\]
Where:
- \(T_{\text{motor}, i}\) is the input torque (`torque_<ID>`).
- \(\omega_{\text{wheel}, i}\) is the wheel angular spin velocity (`wheel_<ID>.ang_vel`).

---

## 2. Output Gap Analysis

To evaluate all these KPIs, we must check if our current FMU exports provide the required variables:

### 2.1. Current FMU Outputs
- **Chassis Reference Frame (`ref_frame`)**: Position, rotation quaternion, linear/angular velocities, and accelerations.
- **Wheel States (`wheel_<ID>`)**: Position, rotation, linear velocity vector, and angular velocity vector.

### 2.2. Gap Analysis Table

| KPI | Required Variables | Currently Exposed? | How to Obtain / Add |
| :--- | :--- | :--- | :--- |
| **Chassis Accel / Yaw Rate** | `ref_frame.pos_dtdt`, `ref_frame.rot_dt` | **Yes** | Fully available from `ref_frame`. |
| **Sideslip Angle** | `ref_frame.pos_dt` | **Yes** | Fully available from `ref_frame`. |
| **Drive Power** | `wheel_<ID>.ang_vel`, `torque_<ID>` | **Yes** | Wheel speed is output; torque is input. |
| **Road Holding / Rollover Index** | Tire vertical forces \(F_z\) | **No** | Need to export tire contact force components from internal tire models. |
| **Tire Wear Index** | Tire forces \(F_x, F_y\), slip velocity | **No** | Need to export tire force vectors and contact slip values. |
| **Active Susp. Power** | Suspension relative velocity \(v_{\text{susp}}\) | **No** | Need to expose suspension travel derivative (velocity) from the suspension links. |
| **Workspace Space Limit** | Suspension displacement \(s_{\text{travel}}\) | **No** | Need to export suspension spring displacement/travel. |

---

## 3. Recommended FMU Output Extensions

To enable comprehensive torque vectoring and active suspension studies, we recommend modifying **`fmu2_wheeled_vehicle_4torques`** to add the following continuous output variables:

1. **Tire Forces (`wheel_<ID>.force`):** 3D vector representing force applied from tire/road interface (`N`).
2. **Suspension Travel (`susp_<ID>.travel`):** Real scalar representing the instantaneous suspension stroke deflection (`m`).
3. **Suspension Velocity (`susp_<ID>.velocity`):** Real scalar representing the relative rate of change of suspension stroke (`m/s`).
4. **Tire Slips (`wheel_<ID>.slip_ratio`, `wheel_<ID>.slip_angle`):** Real values representing longitudinal slip ratio (\(\kappa\)) and slip angle (\(\alpha\)) directly from the tire model calculations.
