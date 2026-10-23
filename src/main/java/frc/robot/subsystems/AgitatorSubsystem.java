// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class AgitatorSubsystem extends SubsystemBase {
  private IntakeSubsystem intake;

  private SparkMax feederAgitator =
      new SparkMax(RobotConstants.kFeederAgitatorCanId, MotorType.kBrushless);

  /** Creates a new FeederSubsystem. */
  public AgitatorSubsystem(IntakeSubsystem intake) {
    this.intake = intake;
  }

  public void converyorRunCmd() {
    feederAgitator.set(RobotConstants.kFeederAgitatorSpeed);
  }

  public void converyorStopCmd() {
    feederAgitator.set(-1 * RobotConstants.kFeederAgitatorSpeed);
    feederAgitator.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
