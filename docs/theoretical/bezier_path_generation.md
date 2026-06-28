# Mathematical Guide: Generating Chrono-Compatible Bezier Paths

This document explains how to convert analytical curves (such as 3rd, 5th, or 7th-degree polynomials) and discrete numerical trajectories into the 9-coordinate Bezier control point format expected by the Chrono Path-Follower Driver FMU.

---

## 1. Chrono Bezier Path Format Overview

In Project Chrono, a path is represented as a composite cubic Bezier spline (`ChBezierCurve`). Each line in the path file corresponds to one node (anchor point) and its incoming/outgoing tangent control points:

$$\text{Line } i: \quad \underbrace{X_i \quad Y_i \quad Z_i}_{\mathbf{P}_i \text{ (Anchor)}} \qquad \underbrace{X_{in} \quad Y_{in} \quad Z_{in}}_{\mathbf{C}_{in} \text{ (In-Control handle)}} \qquad \underbrace{X_{out} \quad Y_{out} \quad Z_{out}}_{\mathbf{C}_{out} \text{ (Out-Control handle)}}$$

* The path **passes exactly** through the anchor point $\mathbf{P}_i$ at the start of segment $i$.
* The shape of the curve between $\mathbf{P}_i$ and $\mathbf{P}_{i+1}$ is governed by $\mathbf{P}_i$'s outgoing control point $\mathbf{C}_{out, i}$ and $\mathbf{P}_{i+1}$'s incoming control point $\mathbf{C}_{in, i+1}$.

---

## 2. Converting a 3rd-Degree Polynomial to Cubic Bezier Segments

Any single-segment cubic polynomial curve can be represented **exactly** as a single cubic Bezier segment. 

### Mathematical Mapping
Suppose you have a 3rd-degree polynomial describing a path $y(x)$ for $x \in [x_0, x_1]$:
$$y(x) = a x^3 + b x^2 + c x + d$$

To convert this polynomial into a Bezier segment running from $t = 0$ (at $x=x_0$) to $t = 1$ (at $x=x_1$), we find the four Bezier points $\mathbf{P}_0$, $\mathbf{C}_0$, $\mathbf{C}_1$, and $\mathbf{P}_1$:

1. **Anchor Points ($\mathbf{P}_0, \mathbf{P}_1$)**:
   The anchor points are the endpoints of the curve:
   $$\mathbf{P}_0 = \begin{bmatrix} x_0 \\ y(x_0) \\ z_0 \end{bmatrix}, \qquad \mathbf{P}_1 = \begin{bmatrix} x_1 \\ y(x_1) \\ z_1 \end{bmatrix}$$

2. **Control Points ($\mathbf{C}_0, \mathbf{C}_1$)**:
   The control points are located along the tangent lines at the endpoints. Let $\Delta x = x_1 - x_0$.
   $$\mathbf{C}_0 = \mathbf{P}_0 + \frac{\Delta x}{3} \begin{bmatrix} 1 \\ y'(x_0) \\ 0 \end{bmatrix} = \begin{bmatrix} x_0 + \frac{\Delta x}{3} \\ y(x_0) + \frac{\Delta x}{3} (3a x_0^2 + 2b x_0 + c) \\ z_0 \end{bmatrix}$$
   $$\mathbf{C}_1 = \mathbf{P}_1 - \frac{\Delta x}{3} \begin{bmatrix} 1 \\ y'(x_1) \\ 0 \end{bmatrix} = \begin{bmatrix} x_1 - \frac{\Delta x}{3} \\ y(x_1) - \frac{\Delta x}{3} (3a x_1^2 + 2b x_1 + c) \\ z_1 \end{bmatrix}$$

This mapping yields the **exact** same geometric trajectory as the analytical cubic equation.

---

## 3. Generating Bezier Control Points for an Arbitrary Numerical Curve

If you have a set of discrete coordinates (e.g., from a test run or numerical solver) without an analytical equation, you can fit a smooth composite cubic Bezier spline.

### Step 1: Obtain the Coordinates
Assume you have $K$ coordinate points:
$$\mathbf{S}_k = \begin{bmatrix} x_k & y_k & z_k \end{bmatrix}^T \quad \text{for } k = 1, \dots, K$$

### Step 2: Compute Tangent Vectors
To ensure a smooth curve (C1 continuity) at each node, the incoming and outgoing control handles must lie on the same tangent line.
1. For an internal point $\mathbf{S}_k$, calculate the tangent vector $\mathbf{T}_k$ using central differences:
   $$\mathbf{T}_k = \frac{\mathbf{S}_{k+1} - \mathbf{S}_{k-1}}{\|\mathbf{S}_{k+1} - \mathbf{S}_{k-1}\|}$$
2. For endpoints, use forward/backward differences:
   $$\mathbf{T}_1 = \frac{\mathbf{S}_2 - \mathbf{S}_1}{\|\mathbf{S}_2 - \mathbf{S}_1\|}, \qquad \mathbf{T}_K = \frac{\mathbf{S}_K - \mathbf{S}_{K-1}}{\|\mathbf{S}_K - \mathbf{S}_{K-1}\|}$$

### Step 3: Position the Control Points
Position the control handles along the tangent lines. A standard spacing is $1/3$ of the distance to the adjacent points:
* **Outgoing handle** for node $k$:
  $$\mathbf{C}_{out, k} = \mathbf{S}_k + \frac{\|\mathbf{S}_{k+1} - \mathbf{S}_k\|}{3} \mathbf{T}_k$$
* **Incoming handle** for node $k$:
  $$\mathbf{C}_{in, k} = \mathbf{S}_k - \frac{\|\mathbf{S}_k - \mathbf{S}_{k-1}\|}{3} \mathbf{T}_k$$

Write these calculated $\mathbf{S}_k$, $\mathbf{C}_{in, k}$, and $\mathbf{C}_{out, k}$ points into your Chrono Bezier file.

---

## 4. Approximating 5th or 7th-Degree Polynomials with Composite Cubic Splines

Since a single cubic Bezier segment is mathematically limited to degree 3, representing a 5th or 7th-degree polynomial path requires dividing the curve into multiple segments (a composite spline).

### Step-by-Step Spline Approximation Algorithm

Suppose you have a 5th-degree polynomial trajectory $y(x) = a x^5 + b x^4 + c x^3 + d x^2 + e x + f$ for $x \in [x_{start}, x_{end}]$.

1. **Subdivide the domain**:
   Choose a spacing interval $\Delta x$ (e.g., $1.0\text{ m}$ for highway curvature, or $0.2\text{ m}$ for tight lane-changes). Construct a vector of node coordinates:
   $$x_k = x_{start} + (k-1)\Delta x \quad \text{for } k=1, \dots, K$$
2. **Evaluate Positions & First Derivatives**:
   At each node $x_k$, evaluate the position and analytical first derivative:
   $$y_k = y(x_k), \qquad y'_k = 5a x_k^4 + 4b x_k^3 + 3c x_k^2 + 2d x_k + e$$
3. **Construct the Bezier File**:
   For each node $k$, define:
   * **Anchor Point**:
     $$\mathbf{P}_k = \begin{bmatrix} x_k & y_k & 0 \end{bmatrix}^T$$
   * **Incoming Control Point** (defined for $k > 1$):
     $$\mathbf{C}_{in, k} = \mathbf{P}_k - \frac{\Delta x}{3} \begin{bmatrix} 1 & y'_k & 0 \end{bmatrix}^T$$
   * **Outgoing Control Point** (defined for $k < K$):
     $$\mathbf{C}_{out, k} = \mathbf{P}_k + \frac{\Delta x}{3} \begin{bmatrix} 1 & y'_k & 0 \end{bmatrix}^T$$
   * *(Note: For the first node $\mathbf{C}_{in, 1}$ and last node $\mathbf{C}_{out, K}$ you can mirror the outgoing/incoming values respectively since they are not used).*

By matching both the positions and the first derivatives at each boundary $x_k$, the composite cubic Bezier spline will approximate your 5th or 7th-degree polynomial with **$G^1$ (geometric tangent) continuity**. The error decreases exponentially with the step size $\Delta x$, making it trivial to reach double-precision accuracy.
