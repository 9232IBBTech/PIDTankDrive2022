// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.PIDMotorSubsystem;

public class PIDPositionCommand extends CommandBase {
  /** Creates a new PIDPositionCommand. */

  private PIDMotorSubsystem pidSubsystem;
  private double target;
  private double error;
  private double allowedErr;
  private boolean shouldEnd;

  public PIDPositionCommand(PIDMotorSubsystem pidSubsystem, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pidSubsystem = pidSubsystem;
    this.target = target;
    allowedErr = target * PIDConstants.ALLOWED_ERROR;

    addRequirements(pidSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidSubsystem.PIDPosition(target);
    // SmartDashboard.putNumber("Allowed Error", allowedErr);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = target - pidSubsystem.getEncoderPosition();
    shouldEnd = Math.abs(error) < allowedErr;
    // SmartDashboard.putNumber("Error", error);
    // SmartDashboard.putBoolean("Should End", shouldEnd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    // pidSubsystem.getMotor().set(0); ihtiyac yok
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd;
  }
}
