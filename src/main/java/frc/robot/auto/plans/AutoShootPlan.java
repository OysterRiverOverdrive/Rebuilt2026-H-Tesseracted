// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.plans;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoConstantShootCmd;
import frc.robot.auto.AutoFeederForwardCmd;
import frc.robot.auto.AutoIntakeForwardCmd;
import frc.robot.auto.AutoSleepCmd;
import frc.robot.commands.intake.IntakeDownCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootPlan extends ParallelCommandGroup {
  /** Creates a new AutoForwardPlan. */
  public AutoShootPlan(FeederSubsystem feeder, ShooterSubsystem shooter, IntakeSubsystem intake) {
    Command lowerIntake = new IntakeDownCommand(intake);

    Command spinIntake = new AutoIntakeForwardCmd(intake, 6);

    Command spinFeeder = new AutoFeederForwardCmd(feeder, 4);

    Command constantShoot = new AutoConstantShootCmd(shooter, 6);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        lowerIntake.andThen(
            spinIntake.alongWith(
                constantShoot.alongWith(new AutoSleepCmd(2).andThen(spinFeeder)))));
  }
}
