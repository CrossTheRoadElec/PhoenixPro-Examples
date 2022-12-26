// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <iostream>

using namespace ctre::phoenixpro;

void Robot::RobotInit() {
  ctre::phoenix::StatusCode returnCode;

  configs::TalonFXConfiguration fxCfg{};
  fxCfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  
  // Apply and loop in case there is an error
  for (auto i = 0; i < 5; i++) {
    returnCode = leftLeader.GetConfigurator().Apply(fxCfg);

    if (returnCode.IsOK()) {
      break;
    } // Otherwise retry up to 5 times
  }

  fxCfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
  
  // Apply and loop in case there is an error
  for (auto i = 0; i < 5; i++) {
    returnCode = rightLeader.GetConfigurator().Apply(fxCfg);

    if (returnCode.IsOK()) {
      break;
    } // Otherwise retry up to 5 times
  }

  configs::CANcoderConfiguration cancoderConfig{};
  cancoderConfig.MagnetSensor.SensorDirection = signals::CANcoder_SensorDirectionValue::CounterClockwise_Positive;
  
  // Apply and loop in case there is an error
  for (auto i = 0; i < 5; i++) {
    returnCode = leftSensor.GetConfigurator().Apply(cancoderConfig);

    if (returnCode.IsOK()) {
      break;
    } // Otherwise try up to 5 times
  }

  cancoderConfig.MagnetSensor.SensorDirection = signals::CANcoder_SensorDirectionValue::Clockwise_Positive;

  // Apply and loop in case there is an error
  for (auto i = 0; i < 5; i++) {
    returnCode = rightSensor.GetConfigurator().Apply(cancoderConfig);

    if (returnCode.IsOK()) {
      break;
    } // Otherwise try up to 5 times
  }

  configs::Pigeon2Configuration pigeonConfig{};

  // Apply and loop in case there is an error
  for (auto i = 0; i < 5; i++) {
    returnCode = imu.GetConfigurator().Apply(pigeonConfig);

    if (returnCode.IsOK()) {
      break;
    } // Otherwise try up to 5 times
  }

  controls::Follower leftFollowerControl{leftLeader.GetDeviceID(), false};
  controls::Follower rightFollowerControl{rightLeader.GetDeviceID(), false};

  leftFollower.SetControl(leftFollowerControl);
  rightFollower.SetControl(rightFollowerControl);

  /* Make sure all critical signals are synchronized */
  /*
   * Setting all these signals to 100hz means they get sent at the same time if
   * they're all on a CANivore
   */
  imu.GetYaw().SetUpdateFrequency(100_Hz);
  leftLeader.GetPosition().SetUpdateFrequency(100_Hz);
  rightLeader.GetPosition().SetUpdateFrequency(100_Hz);

  /* Publish field pose data to read back from */
  frc::SmartDashboard::PutData("Field", &m_field);
}

void Robot::RobotPeriodic() {
  // Update the odometry based on the pigeon 2 rotation and cancoder positions
  m_odometry.Update(imu.GetRotation2d(),
                    rotationsToMeters(leftSensor.GetPosition().GetValue()),
                    rotationsToMeters(rightSensor.GetPosition().GetValue()));

  // Update robot pose for visualization
  m_field.SetRobotPose(m_odometry.GetPose());

  // Print sensor values every 50 loops,
  // this is ~1s assuming 20ms loop time.
  // Users can alternatively publish these values to WPILib SmartDashboard
  if (printCount >= 50) {
    printCount = 0;

    std::cout << "Left FX: " << leftLeader.GetPosition() << std::endl;
    std::cout << "Right FX: " << rightLeader.GetPosition() << std::endl;
    std::cout << "Left CANcoder: " << leftSensor.GetPosition() << std::endl;
    std::cout << "Right CANcoder: " << rightSensor.GetPosition() << std::endl;
    std::cout << "Left Forward limit: " << leftLeader.GetForwardLimit() << std::endl;
    std::cout << "Left Reverse limit: " << leftLeader.GetReverseLimit() << std::endl;
    std::cout << "Right Forward limit: " << rightLeader.GetForwardLimit() << std::endl;
    std::cout << "Right Reverse limit: " << rightLeader.GetReverseLimit() << std::endl;
    std::cout << "Pigeon2: " << imu.GetYaw() << std::endl;
    std::cout << "" << std::endl;
  } else {
    printCount++;
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  drivetrain.CurvatureDrive(-joystick.GetLeftY(), -joystick.GetRightX(), joystick.GetRightStickButton());
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {
  /*
  * CTRE simulation is low-level, so SimState inputs
  * and outputs are not affected by user-level inversion.
  * However, inputs and outputs *are* affected by the mechanical
  * orientation of the device relative to the robot chassis,
  * as specified by the `orientation` field.
  *
  */
  leftSim.Orientation = sim::ChassisReference::CounterClockwise_Positive;
  leftSensSim.Orientation = sim::ChassisReference::CounterClockwise_Positive;

  rightSim.Orientation = sim::ChassisReference::Clockwise_Positive;
  rightSensSim.Orientation = sim::ChassisReference::Clockwise_Positive;
}

void Robot::SimulationPeriodic() {
  /*
  * Set supply voltage to devices. This is used for
  * device closed loop calculations in simulation.
  */
  leftSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  leftSensSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  rightSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  rightSensSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  imuSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  /*
  * WPILib expects positive voltage (+V) to be forward. Since we've
  * already configured the mechanical orientations of our devices, 
  * we do not need to do any additional inversions
  */
  m_driveSim.SetInputs(leftSim.GetMotorVoltage(),
                       rightSim.GetMotorVoltage());

  /*
  * Advance the time of the simulation model by the robot loop time period.
  * This is by default 0.02ms, but users will need to modify this if they've
  * modified the loop timing.
  */
  m_driveSim.Update(20_ms);

  /*
  * WPILib DifferentialDriveSimulation outputs meters while the simulated device
  * expects rotations. Use our conversion function to calculate the raw velocity
  * and position for cancoder.
  */
  auto const leftPos = metersToRotations(m_driveSim.GetLeftPosition());
  auto const leftVel = metersToRotationsVel(m_driveSim.GetLeftVelocity());
  auto const rightPos = metersToRotations(m_driveSim.GetRightPosition());
  auto const rightVel = metersToRotationsVel(m_driveSim.GetRightVelocity());

  // Update sensor outputs
  leftSensSim.SetRawPosition(leftPos);
  leftSensSim.SetVelocity(leftVel);

  rightSensSim.SetRawPosition(rightPos);
  rightSensSim.SetVelocity(rightVel);

  leftSim.SetRawRotorPosition(leftPos * kGearRatio);
  leftSim.SetRotorVelocity(leftVel * kGearRatio);

  rightSim.SetRawRotorPosition(rightPos * kGearRatio);
  rightSim.SetRotorVelocity(rightVel * kGearRatio);

  imuSim.SetRawYaw(m_driveSim.GetHeading().Degrees());

  /*
   * If a bumper is pressed, trigger the forward limit switch to test it,
   * if a trigger is pressed, trigger the reverse limit switch
   */
  leftSim.SetForwardLimit(joystick.GetLeftBumper());
  leftSim.SetReverseLimit(joystick.GetLeftTriggerAxis() > 0.5);
  rightSim.SetForwardLimit(joystick.GetRightBumper());
  rightSim.SetReverseLimit(joystick.GetRightTriggerAxis() > 0.5);
}

units::meter_t Robot::rotationsToMeters(units::turn_t rotations) {
  /* Get circumference of wheel */
  constexpr auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;

  /* Every rotation of the wheel travels this many inches */
  /* Now multiply rotations by meters per rotation */
  return rotations * circumference;
}

units::turn_t Robot::metersToRotations(units::meter_t meters) {
  /* Get circumference of wheel */
  constexpr auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;

  /* Every rotation of the wheel travels this many inches */
  /* Now apply wheel rotations to input meters */
  auto wheelRotations = meters / circumference;

  return wheelRotations;
}

units::meters_per_second_t Robot::rotationsToMetersVel(units::turns_per_second_t rotations) {
  /* Get circumference of wheel */
  constexpr auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;

  /* Every rotation of the wheel travels this many inches */
  /* Now multiply rotations by meters per rotation */
  return rotations * circumference;
}

units::turns_per_second_t Robot::metersToRotationsVel(units::meters_per_second_t meters) {
  /* Get circumference of wheel */
  constexpr auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;

  /* Every rotation of the wheel travels this many inches */
  /* Now apply wheel rotations to input meters */
  auto wheelRotations = meters / circumference;

  return wheelRotations;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
