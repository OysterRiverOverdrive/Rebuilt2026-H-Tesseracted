// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase {
  private boolean isUp = true;

  private SparkMax intakeWheel =
      new SparkMax(RobotConstants.kIntakeWheelCanId, MotorType.kBrushless);
  private SparkMax intakeLift = new SparkMax(RobotConstants.kIntakeLiftCanId, MotorType.kBrushless);

  private SparkMaxConfig liftConfig = new SparkMaxConfig();

  /** Creates a new IntakeWheelSubsystem. */
  public IntakeSubsystem() {
    liftConfig.idleMode(IdleMode.kBrake);

    intakeLift.configure(
        liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intakeWheelForwardCmd() {
    intakeWheel.set(RobotConstants.kIntakeWheelSpeed);
  }

  public void intakeWheelReverseCmd() {
    intakeWheel.set(-1 * RobotConstants.kIntakeWheelSpeed);
  }

  public void intakeWheelStopCmd() {
    intakeWheel.stopMotor();
  }

  public void intakeDownCmd() {
    if (isUp) {
      intakeLift.set(RobotConstants.kIntakeLiftDownSpeed);
      isUp = false;
    }
  }

  // public void intakeUpCmd() {
  //   if(!isUp) {
  //     intakeLift.set(RobotConstants.kIntakeLiftUpSpeed);
  //     isUp = true;
  //   }
  // }

  public void intakeLiftStopCmd() {
    intakeLift.stopMotor();
  }

  public boolean isUp() {
    return isUp;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Is Up", isUp);
    // This method will be called once per scheduler run
  }
}
