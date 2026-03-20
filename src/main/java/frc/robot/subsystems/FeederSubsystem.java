// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class FeederSubsystem extends SubsystemBase {
  private IntakeSubsystem intake;

  private SparkMax feederWheel =
      new SparkMax(RobotConstants.kFeederWheelCanId, MotorType.kBrushless);
  private SparkMax feederSpin = new SparkMax(RobotConstants.kFeederSpinCanId, MotorType.kBrushless);

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem(IntakeSubsystem intake) {
    this.intake = intake;
  }

  public void feederForwardCmd() {
    feederWheel.set(RobotConstants.kFeederWheelSpeed);
    if (!intake.isUp()) {
      feederSpin.set(RobotConstants.kFeederSpinSpeed);
    }
  }

  public void feederReverseCmd() {
    feederWheel.set(-1 * RobotConstants.kFeederWheelSpeed);
    if (!intake.isUp()) {
      feederSpin.set(-1 * RobotConstants.kFeederSpinSpeed);
    }
  }

  public void feederStopCmd() {
    feederWheel.stopMotor();
    feederSpin.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
