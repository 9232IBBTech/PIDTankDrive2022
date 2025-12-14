// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */

  private IntakeSubsystem intake;
  private double triggerVal;
  private boolean isTrigger = true;
  private double currentSpeed;

  public IntakeCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentSpeed = 0;
    if (Constants.ControllerConstants.controller.getRightTriggerAxis() == 0) {
      intake.startIntake(Constants.IntakeConstants.INTAKE_SPEED);
      isTrigger = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isTrigger) {
      triggerVal = Constants.ControllerConstants.controller.getRightTriggerAxis();
      if (triggerVal == 0)
      intake.startIntake(Constants.IntakeConstants.INTAKE_SPEED);

      else {
        triggerVal = (triggerVal > .3 ? triggerVal : 0);
        currentSpeed += intake.rampIntakeInput(currentSpeed, Constants.IntakeConstants.INTAKE_SPEED * triggerVal);
        intake.startIntake(currentSpeed);
      }
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intake.limitSwitch.get();
  }
}
