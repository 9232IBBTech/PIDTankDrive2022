// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.PIDMotorSubsystem;

public class PIDVelocityCommand extends CommandBase {
  /** Creates a new PIDVelocityCommand. */

  private PIDMotorSubsystem pidSubsystem;
  private double target;
  private double allowedErr;

  public PIDVelocityCommand(PIDMotorSubsystem pidSubsystem, double target) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.pidSubsystem = pidSubsystem;
    this.target = target;
    allowedErr = target * PIDConstants.ALLOWED_ERROR;

    addRequirements(pidSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidSubsystem.PIDVelocity(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // pidSubsystem.getMotor().set(pidSubsystem.getEncoderVelocity()); Bunu yapmak istemiyoruz
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(target - pidSubsystem.getEncoderVelocity()) < allowedErr;
  }
}
