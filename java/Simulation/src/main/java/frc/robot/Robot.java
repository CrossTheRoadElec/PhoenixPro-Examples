// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.CANcoder_SensorDirectionValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.sim.CANcoderSimState;
import com.ctre.phoenixpro.sim.ChassisReference;
import com.ctre.phoenixpro.sim.Pigeon2SimState;
import com.ctre.phoenixpro.sim.TalonFXSimState;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private final static String kCanivoreName = "mycanivore";

    private final XboxController joystick = new XboxController(0);

    // All CTRE devices are assumed to be on a canivore bus named "mycanivore"
    private final TalonFX leftLeader = new TalonFX(1, kCanivoreName);
    private final TalonFX rightLeader = new TalonFX(2, kCanivoreName);
    private final TalonFX leftFollower = new TalonFX(3, kCanivoreName);
    private final TalonFX rightFollower = new TalonFX(4, kCanivoreName);

    private final CANcoder leftSensor = new CANcoder(1, kCanivoreName);
    private final CANcoder rightSensor = new CANcoder(2, kCanivoreName);

    private final Pigeon2 imu = new Pigeon2(0, kCanivoreName);

    // Create sim state objects for handling simulation IO
    private final TalonFXSimState leftSim = leftLeader.getSimState();
    private final TalonFXSimState rightSim = rightLeader.getSimState();
    private final CANcoderSimState leftSensSim = leftSensor.getSimState();
    private final CANcoderSimState rightSensSim = rightSensor.getSimState();
    private final Pigeon2SimState imuSim = imu.getSimState();

    private final DifferentialDrive drivetrain = new DifferentialDrive(leftLeader, rightLeader);

    /*
     * These numbers are an example AndyMark Drivetrain with some additional weight.
     * This is a fairly light robot.
     * Note: You can utilize results from robot characterization instead of
     * theoretical numbers.
     * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-
     * characterization/introduction.html#introduction-to-robot-characterization
     */
    private final double kGearRatio = 10.71;
    private final double kWheelRadiusInches = 3;

    /* Simulation model of the drivetrain */
    private final DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500(2), // 2 CIMS on each side of the drivetrain.
        kGearRatio, // Standard AndyMark Gearing reduction.
        2.1, // MOI of 2.1 kg m^2 (from CAD model).
        26.5, // Mass of the robot is 26.5 kg.
        Units.inchesToMeters(kWheelRadiusInches), // Robot uses 3" radius (6" diameter) wheels.
        0.546, // Distance between wheels is _ meters.

        /*
         * The standard deviations for measurement noise:
         * x and y: 0.001 m
         * heading: 0.001 rad
         * l and r velocity: 0.1 m/s
         * l and r position: 0.005 m
         */
        /* Uncomment the following line to add measurement noise. */
        null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    );

    private final Field2d m_field = new Field2d();

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
        imu.getRotation2d(),
        0, 0
    );

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        TalonFXConfiguration fxCfg = new TalonFXConfiguration();
        fxCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Apply and loop in case there is an error
        for (int i = 0; i < 5; i++) {
            var result = leftLeader.getConfigurator().apply(fxCfg);

            if (result.isOK()) {
                break;
            } // Otherwise retry up to 5 times
        }

        fxCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Apply and loop in case there is an error
        for (int i = 0; i < 5; i++) {
            var result = rightLeader.getConfigurator().apply(fxCfg);

            if (result.isOK()) {
                break;
            } // Otherwise retry up to 5 times
        }

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = CANcoder_SensorDirectionValue.CounterClockwise_Positive;

        // apply and loop in case there is an error
        for (int i = 0; i < 5; i++) {
            var result = leftSensor.getConfigurator().apply(cancoderConfig);

            if (result.isOK()) {
                break;
            } // otherwise retry up to 5 times
        }

        cancoderConfig.MagnetSensor.SensorDirection = CANcoder_SensorDirectionValue.Clockwise_Positive;

        // Apply and loop in case there is an error
        for (int i = 0; i < 5; i++) {
            var result = rightSensor.getConfigurator().apply(cancoderConfig);

            if (result.isOK()) {
                break;
            } // Otherwise retry up to 5 times
        }

        var pigeonConfig = new Pigeon2Configuration();

        // Apply and loop in case there is an error
        for (int i = 0; i < 5; i++) {
            var result = imu.getConfigurator().apply(pigeonConfig);

            if (result.isOK()) {
                break;
            } // Otherwise retry up to 5 times
        }
        
        /*
         * Create follower control requests and tell followers to follower their respective leader
         */
        var leftFollowerControl = new Follower(leftLeader.getDeviceID(), false);
        var rightFollowerControl = new Follower(rightLeader.getDeviceID(), false);

        leftFollower.setControl(leftFollowerControl);
        rightFollower.setControl(rightFollowerControl);

        /* Make sure all critical signals are synchronized */
        /*
         * Setting all these signals to 100hz means they get sent at the same time if
         * they're all on a CANivore
         */
        imu.getYaw().setUpdateFrequency(100);
        leftLeader.getPosition().setUpdateFrequency(100);
        rightLeader.getPosition().setUpdateFrequency(100);

        /* Publish field pose data to read back from */
        SmartDashboard.putData("Field", m_field);
    }

    private int printCount = 0;

    @Override
    public void robotPeriodic() {
        // Update the odometry based on the pigeon 2 rotation and cancoder positions
        m_odometry.update(imu.getRotation2d(),
                rotationsToMeters(leftSensor.getPosition().getValue()),
                rotationsToMeters(rightSensor.getPosition().getValue()));

        // Update robot pose for visualization
        m_field.setRobotPose(m_odometry.getPoseMeters());

        // Print sensor values every 50 loops,
        // this is ~1s assuming 20ms loop times.
        // Users can alternatively publish these values to WPILib SmartDashboard
        if (printCount >= 50) {
            printCount = 0;

            System.out.println("Left FX: " + leftLeader.getPosition());
            System.out.println("Right FX: " + rightLeader.getPosition());
            System.out.println("Left CANcoder: " + leftSensor.getPosition());
            System.out.println("Right CANcoder: " + rightSensor.getPosition());
            System.out.println("Left Forward limit: " + leftLeader.getForwardLimit());
            System.out.println("Left Reverse limit: " + leftLeader.getReverseLimit());
            System.out.println("Right Forward limit: " + rightLeader.getForwardLimit());
            System.out.println("Right Reverse limit: " + rightLeader.getReverseLimit());
            System.out.println("Pigeon2: " + imu.getYaw());
            System.out.println();
        } else {
            printCount++;
        }
    }

    @Override
    public void simulationInit() {
        /*
         * CTRE simulation is low-level, so SimState inputs
         * and outputs are not affected by user-level inversion.
         * However, inputs and outputs *are* affected by the mechanical
         * orientation of the device relative to the robot chassis,
         * as specified by the `Orientation` field.
         *
         */
        leftSim.Orientation = ChassisReference.CounterClockwise_Positive;
        leftSensSim.Orientation = ChassisReference.CounterClockwise_Positive;
        
        rightSim.Orientation = ChassisReference.Clockwise_Positive;
        rightSensSim.Orientation = ChassisReference.Clockwise_Positive;
    }

    @Override
    public void simulationPeriodic() {
        /*
         * Set supply voltage to devices. This is used for
         * device closed loop calculations in simulation.
         */
        leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        leftSensSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSensSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        imuSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        /*
        * WPILib expects positive voltage (+V) to be forward. Since we've
        * already configured the mechanical orientations of our devices, 
        * we do not need to do any additional inversions
        */
        m_driveSim.setInputs(leftSim.getMotorVoltage(),
                rightSim.getMotorVoltage());

        /*
         * Advance the time of the simulation model by the robot loop time period.
         * This is by default 0.02ms, but users will need to modify this if they've
         * modified the loop timing.
         */
        m_driveSim.update(TimedRobot.kDefaultPeriod);

        /*
         * WPILib DifferentialDriveSimulation outputs meters while the simulated device
         * expects rotations. Use our conversion function to calculate the raw velocity
         * and position for cancoder.
         */
        final double leftPos = metersToRotations(
                m_driveSim.getLeftPositionMeters());

        final double leftVel = metersToRotations(
                m_driveSim.getLeftVelocityMetersPerSecond());

        final double rightPos = metersToRotations(
                m_driveSim.getRightPositionMeters());
    
        final double rightVel = metersToRotations(
                m_driveSim.getRightVelocityMetersPerSecond());

        // Update sensor outputs
        leftSensSim.setRawPosition(leftPos);
        leftSensSim.setVelocity(leftVel);

        rightSensSim.setRawPosition(rightPos);
        rightSensSim.setVelocity(rightVel);

        leftSim.setRawRotorPosition(leftPos * kGearRatio);
        leftSim.setRotorVelocity(leftVel * kGearRatio);

        rightSim.setRawRotorPosition(rightPos * kGearRatio);
        rightSim.setRotorVelocity(rightVel * kGearRatio);

        imuSim.setRawYaw(m_driveSim.getHeading().getDegrees());

        /*
         * If a bumper is pressed, trigger the forward limit switch to test it, 
         * if a trigger is pressed, trigger the reverse limit switch 
         */
        leftSim.setForwardLimit(joystick.getLeftBumper());
        leftSim.setReverseLimit(joystick.getLeftTriggerAxis() > 0.5);

        rightSim.setForwardLimit(joystick.getRightBumper());
        rightSim.setReverseLimit(joystick.getRightTriggerAxis() > 0.5);
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        drivetrain.curvatureDrive(-joystick.getLeftY(), -joystick.getRightX(), joystick.getRightStickButton());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    private double rotationsToMeters(double rotations) {
        /* Get circumference of wheel */
        final double circumference = kWheelRadiusInches * 2 * Math.PI;

        /* Every rotation of the wheel travels this many inches */
        /* So now get the meters traveled per rotation */
        final double metersPerWheelRotation = Units.inchesToMeters(circumference);
        
        /* Now multiply rotations by meters per rotation */
        return rotations * metersPerWheelRotation;
    }

    private double metersToRotations(double meters) {
        /* Get circumference of wheel */
        final double circumference = kWheelRadiusInches * 2 * Math.PI;

        /* Every rotation of the wheel travels this many inches */
        /* So now get the rotations per meter traveled */

        final double wheelRotationsPerMeter = 1.0 / Units.inchesToMeters(circumference);

        /* Now apply wheel rotations to input meters */
        return wheelRotationsPerMeter * meters;
    }
}
