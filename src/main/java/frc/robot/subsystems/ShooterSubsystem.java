// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private SparkFlex shooterLeft =
      new SparkFlex(RobotConstants.kShooterLeftCanId, MotorType.kBrushless);
  private SparkFlex shooterRight =
      new SparkFlex(RobotConstants.kShooterRightCanId, MotorType.kBrushless);

  private RelativeEncoder shooterEncoder = shooterLeft.getEncoder();

  private SparkClosedLoopController shooterPID;

  private SparkFlexConfig shooterLeftConfig = new SparkFlexConfig();
  private SparkFlexConfig shooterRightConfig = new SparkFlexConfig();

  private double speed;

  public static boolean activePID = false;
  private boolean constantShoot = false;
  private double constantShootValue = ShooterConstants.kShooterConstantSpeed;

  private DrivetrainSubsystem drive;

  public ShooterSubsystem(DrivetrainSubsystem drive) {
    this.drive = drive;

    shooterLeftConfig
        .idleMode(IdleMode.kCoast)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD);

    shooterLeft.configure(
        shooterLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterRightConfig.idleMode(IdleMode.kCoast).follow(shooterLeft, true);

    shooterRight.configure(
        shooterLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterPID = shooterLeft.getClosedLoopController();

    SmartDashboard.putNumber("Used Shoot", 8000);
  }

  public void shooterDislodgeCmd() {
    shooterLeft.set(ShooterConstants.kShooterDislodgeSpeed);
  }

  public void shooterShootCmd() {
    activePID = true;
    constantShoot = false;
  }

  public void shooterConstantShootCmd(double shootValue) {
    activePID = true;
    constantShoot = true;
    constantShootValue = ShooterConstants.kShooterConstantSpeed;
  }

  public void shooterStopCmd() {
    activePID = false;
    shooterPID.setSetpoint(0, ControlType.kVelocity);
    shooterLeft.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", shooterEncoder.getVelocity());

    double distance =
        (Math.abs(
                Math.hypot(
                    DrivetrainSubsystem.getAutoAimTarget().getY() - drive.getVisionPose().getY(),
                    DrivetrainSubsystem.getAutoAimTarget().getX() - drive.getVisionPose().getX())))
            * 100;
    SmartDashboard.putNumber("Distance to Hub", distance);

    if (!constantShoot) {
      if (distance < ShooterConstants.kShooterMaxDistance) {
        SmartDashboard.putBoolean("Can Shoot", true);
        speed = ShooterConstants.getShooterSpeed(distance);
      } else {
        SmartDashboard.putBoolean("Can Shoot", false);
        speed = ShooterConstants.getShooterSpeed(ShooterConstants.kShooterMaxDistance);
      }
    } else {
      speed = SmartDashboard.getNumber("Used Shoot", 8000);
    }

    if (activePID) {
      shooterPID.setSetpoint(-1 * speed, ControlType.kVelocity);
      SmartDashboard.putNumber("Shooting Setpoint", speed);
    }
  }
}
