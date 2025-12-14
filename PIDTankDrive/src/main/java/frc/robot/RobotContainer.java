// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PIDPositionCommand;
import frc.robot.commands.PIDVelocityCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PIDMotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final PIDMotorSubsystem pidSubsystem = new PIDMotorSubsystem();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, Constants.ControllerConstants.controller));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Trigger leftTrigger = new Trigger(() -> Constants.ControllerConstants.controller.getLeftTriggerAxis() > .3);
    Trigger rightTrigger = new Trigger(() -> Constants.ControllerConstants.controller.getRightTriggerAxis() > .3);

    Constants.ControllerConstants.ButtonLB.toggleWhenPressed(new IntakeCommand(intakeSubsystem), true);
    Constants.ControllerConstants.ButtonRB.toggleWhenPressed(new OuttakeCommand(intakeSubsystem), true);
    Constants.ControllerConstants.xButton.whenActive(getAutonomousCommand());
    // Constants.ControllerConstants.bButton.whenActive(new SequentialCommandGroup(new OuttakeCommand(intakeSubsystem), new InstantCommand(() -> {intakeSubsystem.startIntake(0.5);}, intakeSubsystem)));
    // Constants.ControllerConstants.bButton.whenActive(new OuttakeCommand(intakeSubsystem).andThen(new InstantCommand(() -> {intakeSubsystem.startIntake(0.5);}, intakeSubsystem)));

    // Constants.ControllerConstants.bButton.whenActive(new ParallelCommandGroup(new OuttakeCommand(intakeSubsystem), new DriveCommand(driveSubsystem, Constants.ControllerConstants.controller)));
    // Constants.ControllerConstants.bButton.whenActive(new OuttakeCommand(intakeSubsystem).alongWith(new DriveCommand(driveSubsystem, Constants.ControllerConstants.controller)));

    // Constants.ControllerConstants.bButton.whenActive(new ParallelRaceGroup(new OuttakeCommand(intakeSubsystem), new DriveCommand(driveSubsystem, Constants.ControllerConstants.controller)));
    // Constants.ControllerConstants.bButton.whenActive(new ParallelDeadlineGroup(new DriveCommand(driveSubsystem, Constants.ControllerConstants.controller), new OuttakeCommand(intakeSubsystem)));

    Constants.ControllerConstants.bButton.toggleWhenActive(new SequentialCommandGroup(new PIDVelocityCommand(pidSubsystem, 4500), new PrintCommand("Tamam")));
    Constants.ControllerConstants.aButton.whenActive(new InstantCommand(() -> pidSubsystem.zeroEncoder(), pidSubsystem));

    leftTrigger.whileActiveContinuous(new OuttakeCommand(intakeSubsystem));
    rightTrigger.whileActiveContinuous(new IntakeCommand(intakeSubsystem));

    //SequentialCommandGroup a = new SequentialCommandGroup(new OuttakeCommand(intakeSubsystem), new IntakeCommand(intakeSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return intakeSubsystem.PIDControlSmartVelocity();
  }
}
