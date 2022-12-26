// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"
#include "ctre/phoenixpro/Utils.hpp"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>

using namespace ctre::phoenixpro;

DriveSubsystem::DriveSubsystem()
{
    /* Initialize all the devices */
    InitializeLeftDriveTalonFX(m_leftLeader.GetConfigurator());
    InitializeLeftDriveTalonFX(m_leftFollower.GetConfigurator());
    InitializeRightDriveTalonFX(m_rightLeader.GetConfigurator());
    InitializeRightDriveTalonFX(m_rightFollower.GetConfigurator());
    InitializePigeon2(m_pigeon2.GetConfigurator());

    /* Set followers to follow leader */
    m_leftFollower.SetControl(controls::Follower{m_leftLeader.GetDeviceID(), false});
    m_rightFollower.SetControl(controls::Follower{m_rightLeader.GetDeviceID(), false});

    /* Make sure all critical signals are synchronized */
    /* Setting all these signals to 100hz means they get sent at the same time if they're all on a CANivore */
    m_pigeon2.GetYaw().SetUpdateFrequency(100_Hz);
    m_leftLeader.GetPosition().SetUpdateFrequency(100_Hz);
    m_rightLeader.GetPosition().SetUpdateFrequency(100_Hz);

    /* Set the update frequency of the main requests to 0 so updates are sent immediately in the arcadeDrive method */
    m_leftOut.UpdateFreqHz = 0_Hz;
    m_rightOut.UpdateFreqHz = 0_Hz;
    
    /* Currently in simulation, we do not support FOC, so disable it while simulating */
    if (ctre::phoenixpro::IsSimulation())
    {
        m_leftOut.EnableFOC = false;
        m_rightOut.EnableFOC = false;
    }

    /*
     * Set the orientation of the simulated TalonFX devices relative to the robot chassis.
     * WPILib expects +V to be forward. Specify orientations to match that behavior.
     */
    /* left TalonFXs are CCW+ */
    m_leftSimState.Orientation = sim::ChassisReference::CounterClockwise_Positive;
    m_leftFollowerSimState.Orientation = sim::ChassisReference::CounterClockwise_Positive;
    /* right TalonFXs are CW+ */
    m_rightSimState.Orientation = sim::ChassisReference::Clockwise_Positive;
    m_rightFollowerSimState.Orientation = sim::ChassisReference::Clockwise_Positive;

    /* Publish field pose data to read back from */
    frc::SmartDashboard::PutData("Field", &m_field);
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot)
{
    m_leftOut.Output = fwd + rot;
    m_rightOut.Output = fwd - rot;
    m_leftLeader.SetControl(m_leftOut);
    m_rightLeader.SetControl(m_rightOut);
}

void DriveSubsystem::InitializeLeftDriveTalonFX(ctre::phoenixpro::configs::TalonFXConfigurator &cfg)
{
    configs::TalonFXConfiguration toApply{};

    /* User can change configs if they want, or leave this blank for factory-default */
    toApply.MotorOutput.Inverted = false;

    cfg.Apply(toApply);

    /* And initialize position to 0 */
    cfg.SetRotorPosition(0_tr);
}
void DriveSubsystem::InitializeRightDriveTalonFX(ctre::phoenixpro::configs::TalonFXConfigurator &cfg)
{
    configs::TalonFXConfiguration toApply{};

    /* User can change configs if they want, or leave this blank for factory-default */
    toApply.MotorOutput.Inverted = true;

    cfg.Apply(toApply);

    /* And initialize position to 0 */
    cfg.SetRotorPosition(0_tr);
}
void DriveSubsystem::InitializePigeon2(ctre::phoenixpro::configs::Pigeon2Configurator &cfg)
{
    configs::Pigeon2Configuration toApply{};

    /* User can change configs if they want, or leave this blank for factory-default */

    cfg.Apply(toApply);

    /* And initialize yaw to 0 */
    cfg.SetYaw(0_deg);
}

void DriveSubsystem::Periodic()
{
    /*
     * This will get the simulated sensor readings that we set
     * in the previous article while in simulation, but will use
     * real values on the robot itself.
     */
    m_odometry.Update(m_pigeon2.GetRotation2d(),
            rotationsToMeters(m_leftLeader.GetPosition().GetValue()),
            rotationsToMeters(m_rightLeader.GetPosition().GetValue()));
    m_field.SetRobotPose(m_odometry.GetPose());
}

void DriveSubsystem::SimulationPeriodic()
{
    /* Pass the robot battery voltage to the simulated devices */
    m_leftSimState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    m_rightSimState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    m_leftFollowerSimState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    m_rightFollowerSimState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    m_pigeon2SimState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    /*
     * CTRE simulation is low-level, so SimState inputs
     * and outputs are not affected by user-level inversion.
     * However, inputs and outputs *are* affected by the mechanical
     * orientation of the device relative to the robot chassis,
     * as specified by the `orientation` field.
     *
     * WPILib expects +V to be forward. We have already configured
     * our orientations to match this behavior.
     */
    m_driveSim.SetInputs(m_leftSimState.GetMotorVoltage(),
                         m_rightSimState.GetMotorVoltage());

    /*
     * Advance the model by 20 ms. Note that if you are running this
     * subsystem in a separate thread or have changed the nominal
     * timestep of TimedRobot, this value needs to match it.
     */
    m_driveSim.Update(20_ms);

    /* Update all of our sensors. */
    m_leftSimState.SetRawRotorPosition(
        metersToRotations(m_driveSim.GetLeftPosition())
    );
    m_leftSimState.SetRotorVelocity(
        metersToRotationsVel(m_driveSim.GetLeftVelocity())
    );
    m_rightSimState.SetRawRotorPosition(
        metersToRotations(m_driveSim.GetRightPosition())
    );
    m_rightSimState.SetRotorVelocity(
        metersToRotationsVel(m_driveSim.GetRightVelocity())
    );
    m_leftFollowerSimState.SetRawRotorPosition(
        metersToRotations(m_driveSim.GetLeftPosition())
    );
    m_leftFollowerSimState.SetRotorVelocity(
        metersToRotationsVel(m_driveSim.GetLeftVelocity())
    );
    m_rightFollowerSimState.SetRawRotorPosition(
        metersToRotations(m_driveSim.GetRightPosition())
    );
    m_rightFollowerSimState.SetRotorVelocity(
        metersToRotationsVel(m_driveSim.GetRightVelocity())
    );
    m_pigeon2SimState.SetRawYaw(m_driveSim.GetHeading().Degrees());
}

units::meter_t DriveSubsystem::rotationsToMeters(units::turn_t rotations)
{
    /* Get circumference of wheel */
    constexpr auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
    /* Every rotation of the wheel travels this many inches */
    /* Now apply gear ratio to input rotations */
    auto gearedRotations = rotations / kGearRatio;
    /* And multiply geared rotations by meters per rotation */
    return gearedRotations * circumference;
}

units::turn_t DriveSubsystem::metersToRotations(units::meter_t meters)
{
    /* Get circumference of wheel */
    constexpr auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
    /* Every rotation of the wheel travels this many inches */
    /* Now apply wheel rotations to input meters */
    auto wheelRotations = meters / circumference;
    /* And multiply by gear ratio to get rotor rotations */
    return wheelRotations * kGearRatio;
}

units::meters_per_second_t DriveSubsystem::rotationsToMetersVel(units::turns_per_second_t rotations)
{
    /* Get circumference of wheel */
    constexpr auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
    /* Every rotation of the wheel travels this many inches */
    /* Now apply gear ratio to input rotations */
    auto gearedRotations = rotations / kGearRatio;
    /* And multiply geared rotations by meters per rotation */
    return gearedRotations * circumference;
}

units::turns_per_second_t DriveSubsystem::metersToRotationsVel(units::meters_per_second_t meters)
{
    /* Get circumference of wheel */
    constexpr auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
    /* Every rotation of the wheel travels this many inches */
    /* Now apply wheel rotations to input meters */
    auto wheelRotations = meters / circumference;
    /* And multiply by gear ratio to get rotor rotations */
    return wheelRotations * kGearRatio;
}
