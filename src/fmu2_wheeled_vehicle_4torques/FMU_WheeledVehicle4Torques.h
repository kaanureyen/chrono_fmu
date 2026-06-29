// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Antigravity AI
// Repository: https://github.com/kaanureyen/chrono_fmu
// =============================================================================
//
// Co-simulation FMU encapsulating a wheeled vehicle system with 4 wheels.
// The vehicle does not include an engine, transmission, or tires.
// It accepts independent wheel torque inputs directly at the axles.
//
// The wrapped Chrono::Vehicle model is defined through a JSON specification file
// for the vehicle.
//
// This vehicle FMU must be co-simulated with:
//   - 4 independent wheel torque inputs (e.g. from an electric drivetrain or controller)
//   - a driver system which provides steering and braking inputs
//   - 4 tire systems which provide tire loads for each of the 4 wheels (of type
//     TerrainForce).
//
// This vehicle FMU defines continuous output variables for:
//   - vehicle reference frame (of type ChFrameMoving)
//   - wheel states (of type WheelState)
//
// =============================================================================

#pragma once

#include <string>
#include <vector>
#include <array>

#include "chrono/physics/ChLinkTSDA.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_fmi/fmi2/ChFmuToolsExport.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#endif


class VehicleFrictionFunctor;

class FmuComponent : public chrono::fmi2::FmuChronoComponentBase {
    friend class VehicleFrictionFunctor;
  public:
    FmuComponent(fmi2String instanceName,
                 fmi2Type fmuType,
                 fmi2String fmuGUID,
                 fmi2String fmuResourceLocation,
                 const fmi2CallbackFunctions* functions,
                 fmi2Boolean visible,
                 fmi2Boolean loggingOn);
    ~FmuComponent();

    /// Advance dynamics.
    virtual fmi2Status doStepIMPL(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

  private:
    virtual fmi2Status enterInitializationModeIMPL() override;
    virtual fmi2Status exitInitializationModeIMPL() override;

    virtual void preModelDescriptionExport() override;
    virtual void postModelDescriptionExport() override;

    virtual bool is_cosimulation_available() const override { return true; }
    virtual bool is_modelexchange_available() const override { return false; }

    /// Create the vehicle system.
    /// This function is invoked in exitInitializationModeIMPL(), once FMU parameters are set.
    void CreateVehicle();

    /// Configure the vehicle system (tires, terrain, contact formulation).
    /// This function is invoked in exitInitializationModeIMPL(), once FMU parameters are set.
    void ConfigureSystem();

    /// Synchronize the vehicle system with current FMU inputs.
    /// This function is called before advancing dynamics of the vehicle.
    void SynchronizeVehicle(double time);

    /// Extract FMU outputs from the vehicle system.
    /// This function is called after advancing dynamics of the vehicle.
    void CalculateVehicleOutputs();

    /// Functor class to set terrain friction coefficient.
    std::shared_ptr<VehicleFrictionFunctor> friction_functor;

    /// Exchange data for vehicle wheels.
    struct WheelData {
        std::shared_ptr<chrono::vehicle::ChWheel> wheel;
        std::string identifier;
        chrono::vehicle::WheelState state;
        chrono::vehicle::TerrainForce load;
    };

    std::shared_ptr<chrono::vehicle::WheeledVehicle> vehicle;  ///< underlying wheeled vehicle

    // Internal subsystems (embedded tires and terrain)
    std::array<std::shared_ptr<chrono::vehicle::ChTire>, 4> tires;
    std::shared_ptr<chrono::vehicle::ChTerrain> terrain;
    chrono::vehicle::TerrainForces terrain_forces;

#ifdef CHRONO_IRRLICHT
    std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> vis_sys;
#endif

    // FMU I/O parameters
    std::string out_path;  ///< output directory
    fmi2Boolean save_img;         ///< enable/disable saving of visualization snapshots
    double fps;            ///< snapshot saving frequency (in FPS)
    fmi2Boolean m_visible;        ///< visual window setting from instantiation
    fmi2Boolean fmu_visible;      ///< visual window setting from parameter
    fmi2Boolean reset;            ///< reset input to trigger state reset

    // FMU parameters
    std::string data_path;                     ///< path to vehicle data
    std::string resources_dir;                 ///< path to extracted FMU resources
    std::string vehicle_JSON;                  ///< JSON vehicle specification file
    std::string tire_JSON;                     ///< JSON tire specification file
    int terrain_type;                          ///< terrain type (0: Flat, 1: Mesh OBJ, 2: OpenCRG)
    int tire_coll_type;                        ///< tire collision type (0: single, 1: four points, 2: envelope)
    std::string terrain_mesh_file;             ///< OBJ road surface mesh file (for terrain_type = 1)
    std::string terrain_crg_file;              ///< OpenCRG road file (.crg) (for terrain_type = 2)
    double terrain_friction;                   ///< terrain coefficient of friction
    fmi2Boolean system_SMC;                    ///< use SMC contact formulation (NSC otherwise)
    chrono::ChVector3d init_loc;               ///< initial vehicle location
    double init_yaw;                           ///< initial vehicle orientation
    double init_vel;                           ///< initial vehicle forward velocity
    chrono::ChVector3d g_acc;                  ///< gravitational acceleration
    double step_size;                          ///< integration step size

    // FMU continuous inputs and outputs for co-simulation (vehicle-terrain)
    chrono::vehicle::DriverInputs driver_inputs;  ///< vehicle control inputs (input)
    std::array<WheelData, 4> wheel_data;          ///< wheel state and applied forces (output/input)
    chrono::ChFrameMoving<> ref_frame;            ///< vehicle reference frame (output)

    // FMU continuous inputs for 4 independent wheel torques
    double torque_FL;  ///< Front Left wheel torque (input)
    double torque_FR;  ///< Front Right wheel torque (input)
    double torque_RL;  ///< Rear Left wheel torque (input)
    double torque_RR;  ///< Rear Right wheel torque (input)

    // FMU continuous inputs for 4 independent wheel friction coefficients
    double friction_FL;  ///< Front Left wheel friction coefficient (input)
    double friction_FR;  ///< Front Right wheel friction coefficient (input)
    double friction_RL;  ///< Rear Left wheel friction coefficient (input)
    double friction_RR;  ///< Rear Right wheel friction coefficient (input)

    // FMU continuous inputs for active suspension forces
    double act_force_FL;  ///< Front Left active suspension force (input)
    double act_force_FR;  ///< Front Right active suspension force (input)
    double act_force_RL;  ///< Rear Left active suspension force (input)
    double act_force_RR;  ///< Rear Right active suspension force (input)

    // Parallel TSDA actuators
    std::array<std::shared_ptr<chrono::ChLinkTSDA>, 4> active_susp_actuators;

    // FMU continuous outputs for study KPIs (tire forces, slips, suspension travel/velocity)
    std::array<chrono::ChVector3d, 4> tire_force;
    std::array<double, 4> tire_slip_angle;
    std::array<double, 4> tire_slip_ratio;
    std::array<double, 4> susp_travel;
    std::array<double, 4> susp_velocity;

    int render_frame;  ///< counter for rendered frames
    double last_render_time; ///< time of last rendered frame
};
