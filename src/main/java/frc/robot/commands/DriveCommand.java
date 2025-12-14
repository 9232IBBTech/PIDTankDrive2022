package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class DriveCommand extends CommandBase{

    private final DriveSubsystem driveSubsystem;
    private final XboxController controller;
    private double currentForward = 0.0;
    private double currentRotation = 0.0;

    public DriveCommand(DriveSubsystem subsystem, XboxController controller) {
        this.driveSubsystem = subsystem;
        this.controller = controller;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        currentForward = 0.0;
        currentRotation = 0.0;
    }

    @Override
    public void execute() {
        double rawForward = -controller.getLeftY(); // İleri-Geri
        double rawRotation = controller.getRightX(); // Dönme

        double targetForward = applyDeadzone(rawForward, Constants.ControllerConstants.JOYSTICK_DEADZONE);
        double targetRotation = applyDeadzone(rawRotation, Constants.ControllerConstants.JOYSTICK_DEADZONE);

        targetForward *= Constants.TankDriveConstants.ChasisSpeedFactor;
        targetRotation *= Constants.TankDriveConstants.ChasisSpeedFactor;

        currentForward += rampInput(targetForward, currentForward);
        currentRotation += rampInput(targetRotation, currentRotation);

        driveSubsystem.arcadeDrive(currentForward, currentRotation);

        //System.out.println("fr:"+ currentForward);
        //System.out.println("rt:"+ currentRotation);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
    
    private double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) < deadzone ? 0 : value;
    }

    private double rampInput(double targetValue, double currentValue) {
        return (targetValue - currentValue) * Constants.TankDriveConstants.rampRate;
    }
}
