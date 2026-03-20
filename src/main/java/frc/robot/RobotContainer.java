// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.*;
import frc.robot.auto.plans.AutoAllianceZonePlan;
import frc.robot.auto.plans.AutoForwardPlan;
import frc.robot.auto.plans.AutoMiddleFieldPlan;
import frc.robot.auto.plans.AutoShootPlan;
// import frc.robot.auto.plans.*;
import frc.robot.commands.TeleopCmd;
import frc.robot.commands.feeder.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.intake.IntakeWheelForwardCommand;
import frc.robot.commands.intake.IntakeWheelReverseCommand;
import frc.robot.commands.intake.IntakeWheelStopCommand;
import frc.robot.subsystems.*;
import frc.utils.ControllerUtils;
import org.littletonrobotics.urcl.URCL;

public class RobotContainer {
  // Controller Utils Instance
  private final ControllerUtils cutil = new ControllerUtils();

  // Auto Dropdown - Make dropdown variable and variables to be selected
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final String auto1 = "1";
  private final String auto2 = "2";
  private final String auto3 = "3";
  private final String auto4 = "4";
  private final String auto5 = "5";
  private final String auto6 = "6";
  private final String auto7 = "7";
  Command auto;

  // Subsystems
  private final VisionSubsystem vision = new VisionSubsystem();
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(vision);
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem(intake);
  private final ShooterSubsystem shooter = new ShooterSubsystem(drivetrain);

  // Commands
  private final TeleopCmd teleopCmd =
      new TeleopCmd(
          drivetrain,
          () -> cutil.Boolsupplier(Controllers.xbox_lb, DriveConstants.joysticks.DRIVER));

  private static final String[] GIT_FLAG = {"clean", "dirty"};

  // We need to instantiate a Playing With Fusion class in order for their
  // configuration server to be launched on the Rio. This is their recommended
  // minimal approach. See:
  // https://www.chiefdelphi.com/t/2026-playing-with-fusion-product-launch-advanced-battery-solution/507717/115
  private final TimeOfFlight tof = new TimeOfFlight(0);

  // AUTOS
  private final AutoMiddleFieldPlan middleField =
      new AutoMiddleFieldPlan(drivetrain, intake, feeder, shooter);
  private final AutoAllianceZonePlan allianceZone = new AutoAllianceZonePlan(intake);
  private final AutoForwardPlan forward = new AutoForwardPlan(drivetrain, feeder, shooter, intake);
  private final AutoShootPlan shoot = new AutoShootPlan(feeder, shooter, intake);

  public RobotContainer() {

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    URCL.start();

    // log build information
    DataLogManager.log("Git branch: " + BuildConstants.GIT_BRANCH);
    DataLogManager.log("Git date: " + BuildConstants.GIT_DATE);
    DataLogManager.log("Git hash: " + BuildConstants.GIT_SHA);
    DataLogManager.log("Git branch status: " + GIT_FLAG[BuildConstants.DIRTY]);
    DataLogManager.log("Build date: " + BuildConstants.BUILD_DATE);

    // Declare default command during Teleop Period as TeleopCmd(Driving Command)
    drivetrain.setDefaultCommand(teleopCmd);

    // Add Auto options to dropdown and push to dashboard
    m_chooser.setDefaultOption("Do Nothing", auto1);
    m_chooser.addOption("Go Forward", auto2);
    m_chooser.addOption("Middle", auto3);
    m_chooser.addOption("Alliance Side", auto4);
    m_chooser.addOption("Shoot", auto5);
    m_chooser.addOption("Auto[Rename Me]", auto6);
    m_chooser.addOption("Auto[Rename Me]", auto7);
    SmartDashboard.putData("Auto Selector", m_chooser);
    SmartDashboard.putNumber("Auto Wait Time (Sec)", 0);

    // Configure Buttons Methods
    configureBindings();
  }

  private void configureBindings() {
    // Configure buttons
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java
    cutil
        .supplier(Controllers.xbox_rb, DriveConstants.joysticks.DRIVER)
        .onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

    cutil
        .triggerSupplier(Controllers.xbox_rt, 0.3, DriveConstants.joysticks.DRIVER)
        .onTrue(new InstantCommand(() -> drivetrain.recalibrateVisionOdometry()));

    cutil
        .supplier(Controllers.xbox_b, DriveConstants.joysticks.DRIVER)
        .onTrue(new InstantCommand(() -> DrivetrainSubsystem.toggleAutoAim()));

    // Feeder Bindings
    cutil
        .triggerSupplier(Controllers.xbox_rt, 0.2, DriveConstants.joysticks.OPERATOR)
        .onTrue(new FeederForwardCommand(feeder))
        .onFalse(new FeederStopCommand(feeder));

    cutil
        .supplier(Controllers.xbox_rb, DriveConstants.joysticks.OPERATOR)
        .onTrue(new FeederReverseCommand(feeder))
        .onFalse(new FeederStopCommand(feeder));

    // Intake Wheel Bindings
    cutil
        .triggerSupplier(Controllers.xbox_lt, 0.2, DriveConstants.joysticks.OPERATOR)
        .onTrue(new IntakeWheelForwardCommand(intake))
        .onFalse(new IntakeWheelStopCommand(intake));

    cutil
        .supplier(Controllers.xbox_lb, DriveConstants.joysticks.OPERATOR)
        .onTrue(new IntakeWheelReverseCommand(intake))
        .onFalse(new IntakeWheelStopCommand(intake));

    // Shooter Bindings
    cutil
        .supplier(Controllers.xbox_a, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> shooter.shooterDislodgeCmd()))
        .onFalse(new InstantCommand(() -> shooter.shooterStopCmd()));

    cutil
        .supplier(Controllers.xbox_b, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> shooter.shooterShootCmd()))
        .onFalse(new InstantCommand(() -> shooter.shooterStopCmd()));

    cutil
        .supplier(Controllers.xbox_y, DriveConstants.joysticks.OPERATOR)
        .onTrue(new InstantCommand(() -> shooter.shooterConstantShootCmd()))
        .onFalse(new InstantCommand(() -> shooter.shooterStopCmd()));

    // Intake Lift Bindings
    cutil
        .supplier(Controllers.xbox_x, DriveConstants.joysticks.OPERATOR)
        .onTrue(new IntakeDownCommand(intake));

    // cutil
    //     .supplier(Controllers.xbox_y, DriveConstants.joysticks.OPERATOR)
    //     .onTrue(new IntakeUpCommand(intake));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // Return NOTHING, replace with command to be run in autonomous period
    // Prior Reference:
    // https://github.com/OysterRiverOverdrive/Charged-Up-2023-Atlas_Chainsaw/blob/main/src/main/java/frc/robot/RobotContainer.java

    switch (m_chooser.getSelected()) {
      default:
      case auto1:
        auto = new BeginSleepCmd(drivetrain, 0);
        break;
      case auto2:
        auto = forward;
        break;
      case auto3:
        auto = middleField;
        break;
      case auto4:
        auto = allianceZone;
        break;
      case auto5:
        auto = shoot;
        break;
      case auto6:
        break;
      case auto7:
        break;
    }
    // Create sequential command with the wait command first then run selected auto
    auto =
        new SequentialCommandGroup(
            new BeginSleepCmd(drivetrain, SmartDashboard.getNumber("Auto Wait Time (Sec)", 0)),
            auto);
    return auto;
  }
}
