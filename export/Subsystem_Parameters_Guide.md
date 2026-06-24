# Chrono Vehicle Subsystem Parameters Guide

This guide documents all parameters and templates for the JSON configuration files used by the 4-wheel vehicle FMU (`FMU2cs_WheeledVehicle4Torques`). 
All units are in **SI units** (meters, kilograms, seconds, newtons, radians) unless explicitly specified otherwise.

---

## Table of Contents
1. [Vehicle.json](#1-vehiclejson-wheeledvehicle)
2. [Chassis.json](#2-chassisjson-rigidchassis)
3. [DoubleWishboneFront.json / DoubleWishboneRear.json](#3-doublewishbonefrontjson--doublewishbonerearjson-doublewishbone)
4. [Wheel.json](#4-wheeljson-wheel)
5. [BrakeSimple_Front.json / BrakeSimple_Rear.json](#5-brakesimple_frontjson--brakesimple_rearjson-brakesimple)
6. [RackPinion.json](#6-rackpinionjson-rackpinion)
7. [Driveline2WD.json](#7-driveline2wdjson-shaftsdriveline2wd)
8. [Sedan_Pac02Tire.json](#8-sedan_pac02tirejson-pac02tire)
9. [Sedan_TMeasyTire.json](#9-sedan_tmeasytirejson-tmeasytire)

---

## 1. Vehicle.json (WheeledVehicle)
Defines the assembly of the vehicle, linking all its subsystems (chassis, suspensions, steering, wheels, brakes, driveline).

### Parameters
*   `Name` (string): Descriptive name of the vehicle.
*   `Type` (string): Must be `"Vehicle"`.
*   `Template` (string): Must be `"WheeledVehicle"`.
*   `Chassis` (object):
    *   `Input File` (string): Path to the chassis JSON file (e.g. `"Chassis.json"`).
    *   `Output` (bool, optional): Enable outputting chassis states to files. Default is `false`.
*   `Rear Chassis` (array of objects, optional): For articulated vehicles.
    *   `Input File` (string): Rear chassis JSON file.
    *   `Connector Input File` (string): Connection joint specification JSON file.
    *   `Chassis Index` (int): Index of the chassis this one attaches to.
*   `Subchassis` (array of objects, optional):
    *   `Input File` (string): Subchassis JSON specification.
    *   `Subchassis Location` (vector3: `[X, Y, Z]`): Location relative to vehicle reference frame.
    *   `Chassis Index` (int): Parent chassis index.
*   `Steering Subsystems` (array of objects):
    *   `Input File` (string): Steering JSON specification (e.g. `"RackPinion.json"`).
    *   `Location` (vector3): Steering subsystem coordinates.
    *   `Orientation` (quaternion: `[e0, e1, e2, e3]`): Orientation of the steering subsystem.
*   `Driveline` (object):
    *   `Input File` (string): Driveline JSON specification (e.g. `"Driveline2WD.json"`).
    *   `Suspension Indexes` (array of ints): Index of axles driven by this driveline (e.g. `[1]` for Rear-Wheel Drive).
*   `Axles` (array of objects): List of axles (typically index 0 for Front, index 1 for Rear).
    *   `Suspension Input File` (string): Suspension JSON specification.
    *   `Suspension Location` (vector3): Suspension mounting location relative to vehicle frame.
    *   `Steering Index` (int, optional): Index of the steering subsystem controlling this axle. Default is `-1` (non-steerable).
    *   `Chassis Index` (int, optional): Parent chassis index. Default is `0`.
    *   `Subchassis Index` (int, optional): Attaching subchassis index. Default is `-1`.
    *   `Antirollbar Input File` (string, optional): Antirollbar JSON file path.
    *   `Antirollbar Location` (vector3, optional): Mounting coordinates (required if anti-roll bar is present).
    *   *For single-wheel axles (Default)*:
        *   `Left Wheel Input File` (string): Left wheel JSON specification.
        *   `Right Wheel Input File` (string): Right wheel JSON specification.
    *   *For double-wheel axles*:
        *   `Wheel Separation` (double): Lateral distance between double wheel centers.
        *   `Left Inside Wheel Input File` (string)
        *   `Right Inside Wheel Input File` (string)
        *   `Left Outside Wheel Input File` (string)
        *   `Right Outside Wheel Input File` (string)
    *   `Left Brake Input File` (string, optional): Left brake JSON specification.
    *   `Right Brake Input File` (string, optional): Right brake JSON specification.
*   `Wheelbase` (double, optional): Vehicle wheelbase. If omitted, calculated automatically from suspension locations.
*   `Minimum Turning Radius` (double, optional): Expected minimum turning radius.
*   `Maximum Steering Angle (deg)` (double, optional): Maximum steering deflection in degrees.

---

## 2. Chassis.json (RigidChassis)
Defines the mass properties, driver location, and collision/visualization models of the rigid chassis.

### Parameters
*   `Name` (string): Descriptive name.
*   `Type` (string): Must be `"Chassis"`.
*   `Template` (string): Must be `"RigidChassis"`.
*   `Components` (array of objects): Mass segments comprising the chassis.
    *   `Centroidal Frame` (object):
        *   `Location` (vector3): Location of component COM relative to chassis coordinate system.
        *   `Orientation` (quaternion): Component orientation.
    *   `Mass` (double): Component mass in kg.
    *   `Moments of Inertia` (vector3: `[Ixx, Iyy, Izz]`): Principal moments of inertia.
    *   `Products of Inertia` (vector3: `[Ixy, Ixz, Iyz]`): Products of inertia.
    *   `Void` (bool): If `true`, the mass/inertia properties are subtracted instead of added.
*   `Driver Position` (object):
    *   `Location` (vector3): Coordinates of the driver seat relative to chassis frame.
    *   `Orientation` (quaternion): Orientation of the driver seat.
*   `Rear Connector Location` (vector3, optional): Mounting point coordinate for rear trailer connectors.
*   `Contact` (object, optional): Define collision geometry if the chassis collides with obstacles/terrain.
    *   `Materials` (array of objects): List of contact material parameters.
        *   `Coefficient of Friction` (float)
        *   `Coefficient of Restitution` (float)
        *   `Properties` (object, optional):
            *   `Young Modulus` (float)
            *   `Poisson Ratio` (float)
        *   `Coefficients` (object, optional):
            *   `Normal Stiffness` (float), `Normal Damping` (float), `Tangential Stiffness` (float), `Tangential Damping` (float)
    *   `Shapes` (array of objects): Collision shapes.
        *   `Type` (string): `"SPHERE"`, `"BOX"`, `"CYLINDER"`, `"HULL"`, or `"MESH"`.
        *   `Material Index` (int): Index referring to `Materials` array above.
        *   `Location` (vector3): Location of shape.
        *   `Orientation` (quaternion, for BOX/CYLINDER)
        *   `Radius` (double, for SPHERE/CYLINDER)
        *   `Dimensions` (vector3: `[X, Y, Z]`, for BOX)
        *   `Axis` (vector3, for CYLINDER axis direction)
        *   `Length` (double, for CYLINDER length)
        *   `Filename` (string, path to OBJ mesh for HULL/MESH shapes)
        *   `Contact Radius` (double, for MESH contact thickness)
*   `Visualization` (object, optional):
    *   `Mesh` (string, optional): Path to visual OBJ model (e.g. `"vehicle/sedan/sedan_chassis_vis.obj"`).
    *   `Primitives` (array of objects, optional): Visual primitive shapes (using SPHERE, BOX, CYLINDER with same format as contact shapes but without `Material Index`).

---

## 3. DoubleWishboneFront.json / DoubleWishboneRear.json (DoubleWishbone)
Defines a double wishbone suspension, including control arms, uprights, spindles, tierods, springs, dampers, and bushings.

### Parameters
*   `Name`, `Type` (`"Suspension"`), `Template` (`"DoubleWishbone"`).
*   `Camber Angle (deg)` (double, optional): Static camber angle. Default is `0`.
*   `Toe Angle (deg)` (double, optional): Static toe angle. Default is `0`.
*   `Vehicle-Frame Inertia` (bool, optional): If `true`, moments/products of inertia for control arms/uprights are assumed in the vehicle-aligned frame. Default is `false`.
*   `Spindle` (object): Assembly representing the wheel hub/spindle.
    *   `Mass` (double), `COM` (vector3), `Inertia` (vector3), `Radius` (double), `Width` (double).
*   `Upright` (object): Upright connecting spindle to control arms.
    *   `Mass`, `COM`, `Moments of Inertia`, `Products of Inertia`, `Radius`.
*   `Upper Control Arm` (object) & `Lower Control Arm` (object):
    *   `Mass`, `COM`, `Moments of Inertia`, `Products of Inertia`, `Radius`.
    *   `Location Chassis Front` (vector3): Front mounting point on chassis.
    *   `Location Chassis Back` (vector3): Rear mounting point on chassis.
    *   `Location Upright` (vector3): Joint connection point to the upright.
    *   `Bushing Data` (object, optional): Elastomeric bushing parameters instead of spherical/revolute joints.
        *   `Stiffness Linear` (double), `Damping Linear` (double)
        *   `Stiffness Rotational` (double), `Damping Rotational` (double)
*   `Tierod` (object): steering tie-rod connecting rack or chassis to upright.
    *   `Location Chassis` (vector3), `Location Upright` (vector3).
    *   *To enable Tierod as a physical body with mass (Optional)*:
        *   `Mass` (double), `Inertia` (vector3), `Radius` (double).
        *   `Bushing Data` (object, optional): Bushing properties for tierod joints.
*   `Spring` (object): Spring elements (TSDA) connecting arm to chassis.
    *   `Location Chassis` (vector3): Upper mounting point.
    *   `Location Arm` (vector3): Lower mounting point.
    *   `Free Length` (double): Unloaded spring length.
    *   *For Linear Spring (Default)*:
        *   `Spring Coefficient` (double): Linear stiffness constant.
        *   `Preload` (double, optional): Force preload.
        *   `Minimum Length` / `Maximum Length` (double, optional): Mechanical travel limits (bump stops).
    *   *For Nonlinear Spring (Optional)*:
        *   `Spring Curve Data` (array of 2D double arrays): Displacement-force lookup table `[[displacement, force], ...]`.
        *   `Preload` (double, optional)
*   `Shock` (object): Damper elements (TSDA).
    *   `Location Chassis` (vector3): Upper mounting point.
    *   `Location Arm` (vector3): Lower mounting point.
    *   *For Linear Damper (Default)*:
        *   `Damping Coefficient` (double): Linear damping constant.
    *   *For Degressive Damper (Optional)*:
        *   `Damping Coefficient` (double): Initial low-speed damping slope.
        *   `Degressivity Compression` (double): High-speed flattening coefficient in compression.
        *   `Degressivity Expansion` (double): High-speed flattening coefficient in expansion.
    *   *For Nonlinear Damper (Optional)*:
        *   `Damping Curve Data` (array of 2D double arrays): Velocity-force lookup table `[[velocity, force], ...]`.
*   `Axle` (object):
    *   `Inertia` (double): Rotational inertia of the drive axle shaft in kg·m².

---

## 4. Wheel.json (Wheel)
Defines wheel mass, rotational inertia, and visualization parameters.

### Parameters
*   `Name`, `Type` (`"Wheel"`), `Template` (`"Wheel"`).
*   `Mass` (double): Wheel mass (excluding tire) in kg.
*   `Inertia` (vector3: `[Ixx, Iyy, Izz]`): Moments of inertia.
*   `Visualization` (object, optional):
    *   `Mesh Filename` (string): Path to visual OBJ model (e.g. `"vehicle/sedan/sedan_rim.obj"`).
    *   `Radius` (double): Outer rim radius.
    *   `Width` (double): Rim width.

---

## 5. BrakeSimple_Front.json / BrakeSimple_Rear.json (BrakeSimple)
Defines a simple brake model where brake torque is proportional to the braking input.

### Parameters
*   `Name`, `Type` (`"Brake"`), `Template` (`"BrakeSimple"`).
*   `Maximum Torque` (double): Maximum braking torque applied when brake input is `1.0` (in Nm).

---

## 6. RackPinion.json (RackPinion)
Defines a rack-and-pinion steering mechanism.

### Parameters
*   `Name`, `Type` (`"Steering"`), `Template` (`"RackPinion"`).
*   `Steering Link` (object): Properties of the steering rack bar.
    *   `Mass` (double), `COM` (double: offset along Y-axis), `Inertia` (vector3), `Radius` (double), `Length` (double).
*   `Pinion` (object): Pinion gear properties.
    *   `Radius` (double): Pinion radius (defines linear rack displacement per steering angle).
    *   `Maximum Angle (deg)` (double): Limit of pinion rotational angle.

---

## 7. Driveline2WD.json (ShaftsDriveline2WD)
Defines a 2WD (Front or Rear-Wheel Drive) powertrain connection using rotational shafts.

### Parameters
*   `Name`, `Type` (`"Driveline"`), `Template` (`"ShaftsDriveline2WD"`).
*   `Shaft Direction` (object): Rotation direction vectors.
    *   `Motor Block` (vector3: `[X, Y, Z]`): Engine crankshaft rotation axis.
    *   `Axle` (vector3: `[X, Y, Z]`): Axle drive shaft rotation axis.
*   `Shaft Inertia` (object): Shaft rotational inertias.
    *   `Driveshaft` (double): Driveshaft rotational inertia.
    *   `Differential Box` (double): Differential cage rotational inertia.
*   `Gear Ratio` (object): Gear reductions.
    *   `Conical Gear` (double): Gear ratio of bevel/ring gear differential.
*   `Axle Differential Locking Limit` (double, optional): Locking torque bias limit for a limited slip differential. Default is `100`.

---

## 8. Sedan_Pac02Tire.json (Pac02Tire)
Defines the Pacejka 2002 Magic Formula tire.

### Parameters
*   `Name`, `Type` (`"Tire"`), `Template` (`"Pac02Tire"`).
*   `Mass` (double): Tire mass in kg.
*   `Inertia` (vector3: `[Ixx, Iyy, Izz]`): Rotational inertia.
*   `Coefficient of Friction` (double): Reference surface coefficient of friction.
*   *Option A: Loading from external TIR file (Default)*:
    *   `TIR Specification File` (string): Path to Adams/Car `.tir` file (e.g. `"vehicle/sedan/tire/Sedan_Pac02Tire.tir"`).
*   *Option B: Defining parameters directly in JSON*:
    *   `Use Mode` (int): Tire simulation mode (0: vertical only, 1: longitudinal, 2: lateral, 3: uncoupled, 4: combined).
    *   `Tire Side` (string): `"left"` or `"right"`.
    *   `Friction Ellipsis Mode` (bool)
    *   `FITTYP` (int): Pacejka fit version (typically 5 or 6).
    *   `VXLOW` (double): Low speed threshold.
    *   `LONGVL` (double): Nominal slip velocity.
    *   `Tire Conditions` (object):
        *   `Inflation Pressure` (double), `Nominal Inflation Pressure` (double).
    *   `Dimension` (object):
        *   `Unloaded Radius` (double), `Width` (double), `Aspect Ratio` (double), `Rim Radius` (double), `Rim Width` (double).
*   `Visualization` (object, optional):
    *   `Mesh Filename Left` / `Mesh Filename Right` (string): Paths to left/right tire visual models.
    *   `Width` (double): Visual tire width.

---

## 9. Sedan_TMeasyTire.json (TMeasyTire)
Defines the TMeasy (Tire Model Easy) passenger/truck tire.

### Parameters
*   `Name`, `Type` (`"Tire"`), `Template` (`"TMeasyTire"`).
*   `Design` (object): Basic dimensions and mass.
    *   `Mass [kg]` (double)
    *   `Inertia [kg.m2]` (vector3)
    *   `Unloaded Radius [m]` (double)
    *   `Rim Radius [m]` (double)
    *   `Width [m]` (double)
*   `Coefficient of Friction` (double): Reference friction coefficient.
*   `Rolling Resistance Coefficient` (double): Tire rolling resistance constant.
*   `Inflation Pressure Design [Pa]` & `Inflation Pressure Use [Pa]` (double, optional).
*   *Option A: Automatic Generation via Load Index (Default)*:
    *   `Load Index` (unsigned int): Tire load index code (e.g. `97`).
    *   `Vehicle Type` (string): `"Passenger"` or `"Truck"`.
*   *Option B: Automatic Generation via Bearing Capacity*:
    *   `Maximum Bearing Capacity [N]` (double): Maximum load rating in Newtons.
    *   `Vehicle Type` (string): `"Passenger"` or `"Truck"`.
*   *Option C: Full Manual Parametrization (Parameters block)*:
    *   If you wish to fully specify vertical, longitudinal, lateral, and aligning stiffness/force curves manually, refer to the Chrono TMeasy source documentation for the full `"Parameters"` sub-block.
*   `Visualization` (object, optional):
    *   `Mesh Filename Left` / `Mesh Filename Right` (string): Paths to visual tire models.
    *   `Width` (double): Visual width.
