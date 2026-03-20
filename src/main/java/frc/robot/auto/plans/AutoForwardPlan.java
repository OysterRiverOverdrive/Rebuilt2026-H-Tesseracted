// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.auto.AutoDistanceShootCmd;
import frc.robot.auto.AutoFeederForwardCmd;
import frc.robot.auto.AutoSleepCmd;
import frc.robot.commands.intake.IntakeDownCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoForwardPlan extends ParallelCommandGroup {
  /** Creates a new AutoForwardPlan. */
  public AutoForwardPlan(
      DrivetrainSubsystem drive,
      FeederSubsystem feeder,
      ShooterSubsystem shooter,
      IntakeSubsystem intake) {
    Command goForward =
        AutoCreationCmd.AutoRobotDriveCmd(
            drive,
            List.of(new Translation2d(1.0, 0.0)),
            new Pose2d(2.0, 0.0, new Rotation2d(-Math.PI / 4)));

    Command lowerIntake = new IntakeDownCommand(intake);

    Command spinFeeder = new AutoFeederForwardCmd(feeder, 4);

    Command distanceShoot = new AutoDistanceShootCmd(shooter, 6);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        goForward
            .andThen(lowerIntake)
            .andThen(distanceShoot.alongWith(new AutoSleepCmd(2).andThen(spinFeeder))));
  }
}
