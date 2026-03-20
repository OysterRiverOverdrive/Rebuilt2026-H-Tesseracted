// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 4.8 / 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = (2 * Math.PI) / 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 6;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
    public static final double kPXController = 8;
    public static final double kPYController = 8;
    public static final double kPThetaController = 5;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static class VisionConstants {
    // Offsets, not measured so a transform of zero
    // Positive x is forward, positive y is left, positive z is up
    // Our official configuration has, front left is camera 1, and back left is camera 2,
    // back right is camera 3, front right is camera 4
    public static final Transform3d kRobotToCam1 =
        new Transform3d(
            Distance.ofRelativeUnits(11.5, Units.Inches),
            Distance.ofRelativeUnits(11.5, Units.Inches),
            Distance.ofRelativeUnits(9.5, Units.Inches),
            new Rotation3d(
                Angle.ofRelativeUnits(0, Units.Degrees),
                Angle.ofRelativeUnits(15, Units.Degrees),
                Angle.ofRelativeUnits(45, Units.Degrees)));

    public static final Transform3d kRobotToCam2 =
        new Transform3d(
            Distance.ofRelativeUnits(-11.5, Units.Inches),
            Distance.ofRelativeUnits(11.5, Units.Inches),
            Distance.ofRelativeUnits(9.5, Units.Inches),
            new Rotation3d(
                Angle.ofRelativeUnits(0, Units.Degrees),
                Angle.ofRelativeUnits(15, Units.Degrees),
                Angle.ofRelativeUnits(135, Units.Degrees)));

    public static final Transform3d kRobotToCam3 =
        new Transform3d(
            Distance.ofRelativeUnits(-11.5, Units.Inches),
            Distance.ofRelativeUnits(-11.5, Units.Inches),
            Distance.ofRelativeUnits(9.5, Units.Inches),
            new Rotation3d(
                Angle.ofRelativeUnits(0, Units.Degrees),
                Angle.ofRelativeUnits(15, Units.Degrees),
                Angle.ofRelativeUnits(225, Units.Degrees)));

    public static final Transform3d kRobotToCam4 =
        new Transform3d(
            Distance.ofRelativeUnits(11.5, Units.Inches),
            Distance.ofRelativeUnits(-11.5, Units.Inches),
            Distance.ofRelativeUnits(9.5, Units.Inches),
            new Rotation3d(
                Angle.ofRelativeUnits(0, Units.Degrees),
                Angle.ofRelativeUnits(15, Units.Degrees),
                Angle.ofRelativeUnits(315, Units.Degrees)));

    public static final int kNotStaleTime = 3;
    public static final int kOdometryUpdateFrequency = 10;

    // Note that this is used to divide, making vision trusted more
    public static final double kVisionOdometryStandardDevScalar = 5;

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final Translation2d kBlueAllianceHub =
        new Translation2d(
            Distance.ofRelativeUnits(182.105, Units.Inches),
            Distance.ofRelativeUnits(158.84, Units.Inches));

    public static final Translation2d kRedAllianceHub =
        new Translation2d(
            Distance.ofRelativeUnits(469.115, Units.Inches),
            Distance.ofRelativeUnits(158.84, Units.Inches));

    public static final double kAutoAimP = 1.2;
  }

  // Constants specifically for Driving & Operation
  public static class DriveConstants {
    // Controller Ports ---
    // Determined here but assigned in the driver station to determine and organize physical ports
    // the controllers are plug into
    public static final int kDrveControllerPort = 0;
    public static final int kOperControllerPort = 1;

    // Driver Controller Joystick ---
    public static final int kDriveX = 0;
    public static final int kDriveY = 1;
    public static final int kDriveRotate = 4;
    public static final double deadzoneDriver = 0.12;

    public enum joysticks {
      DRIVER,
      OPERATOR
    }

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds

    public static final double kMaxSpeedMetersPerSecond = 15;
    public static final double kMaxAngularSpeed = 1.25 * 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 4; // radians per second
    public static final double kMagnitudeSlewRate = 2; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 5; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth =
        Distance.ofRelativeUnits(26.5, Units.Inches).in(Units.Meters);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase =
        Distance.ofRelativeUnits(26.5, Units.Inches).in(Units.Meters);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  }

  // Constants specifically for the physical robot
  public static final class RobotConstants {
    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kFrontRightTurningCanId = 2;

    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kFrontLeftTurningCanId = 4;

    public static final int kRearRightDrivingCanId = 7;
    public static final int kRearRightTurningCanId = 6;

    public static final int kRearLeftDrivingCanId = 9;
    public static final int kRearLeftTurningCanId = 8;

    public static final int kIntakeLiftCanId = 15;
    public static final int kIntakeWheelCanId = 16;
    public static final int kShooterMainCanId = 17;
    public static final int kFeederWheelCanId = 19;
    public static final int kFeederSpinCanId = 20;
    public static final int kClimberCanId = 21;

    // Used to declare Navx as upside down
    public static final boolean kGyroReversed = false;

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final double kIntakeLiftDownSpeed = 0.1;
    public static final double kIntakeLiftDownDuration = 0.3;
    public static final double kIntakeLiftUpSpeed = -1;
    public static final double kIntakeLiftUpDuration = 2;
    public static final double kIntakeWheelSpeed = -0.6;
    public static final double kFeederWheelSpeed = 0.3;
    public static final double kFeederSpinSpeed = 0.3;
    public static final double kClimberSpeed = 0.3;
  }

  public static final class ShooterConstants {
    public static final double kShooterDislodgeSpeed = 0.2;

    // Maximum distance the shooter can shoot at in centimeters
    public static final double kShooterMaxDistance = 850;

    /**
     * Calculates the speed to give the shooter based on distance from the hub
     *
     * @param distance the robot's distance form the hub in centimeters
     * @return the speed to give the shooter
     */
    public static double getShooterSpeed(double distance) {
      return ((12.84983 * distance) + 4875);
    }

    public static final double kShooterConstantSpeed = 10000;

    public static final double kShooterSpecializedConstantSpeed = getShooterSpeed(2.5);

    public static final double kShooterP = 0.0001;
    public static final double kShooterI = 0;
    public static final double kShooterD = 0;
  }

  // Constants specifically for Swerve Module
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second
    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    // PID Driving Values ---
    // Most likely used to act as a form of slew
    // See:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMaxOutput = 1;
    public static final double kDrivingMinOutput = kDrivingMaxOutput * -1;
    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMaxOutput = 1;
    public static final double kTurningMinOutput = kTurningMaxOutput * -1;

    // Swerve Module Idle Modes ---
    // These determine how the module behavior when there is a lack of input from the driver
    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    // Current limits ---
    // Meant for the electrical side of the drivetrain to make sure that the drivetrain isn't
    // drawing too much power
    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 15; // amps
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
