// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.plans;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoIntakeForwardCmd;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoAllianceZonePlan extends ParallelCommandGroup {
  // subsystems

  public AutoAllianceZonePlan(IntakeSubsystem intake) {
    Command spinIntake = // spin the wheel kronk
        new AutoIntakeForwardCmd(intake, 5);

    addCommands(spinIntake);
  }
}
