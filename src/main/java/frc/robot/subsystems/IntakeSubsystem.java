// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem2. */

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private MotorControllerGroup systemMotors;
  private SparkMaxPIDController leftPIDController;
  private RelativeEncoder leftEncoder; 
  private int pidSlotID = 0;
  private int smartPIDSlotID = 1;
  private boolean reachedTarget = false;

  public DigitalInput limitSwitch;

  public IntakeSubsystem() {

    leftMotor = new CANSparkMax(IntakeConstants.INTAKE_RIGHT_MOTOR, MotorType.kBrushless); //LEFT
    rightMotor = new CANSparkMax(IntakeConstants.INTAKE_LEFT_MOTOR, MotorType.kBrushless); //RIGHT

    leftMotor.restoreFactoryDefaults();
    leftEncoder = leftMotor.getEncoder();
    leftEncoder.setPosition(0);
    leftEncoder.setPositionConversionFactor(Constants.IntakeConstants.GEAR_RATIO);

    leftPIDController = leftMotor.getPIDController();
    leftPIDController.setSmartMotionMaxVelocity(Constants.MotorConstants.NEO_MAX_VEL - 1000, smartPIDSlotID);
    leftPIDController.setSmartMotionMaxAccel(3500, smartPIDSlotID);
    leftPIDController.setSmartMotionAllowedClosedLoopError(2, smartPIDSlotID);

    leftPIDController.setP(0.0023, pidSlotID);
    leftPIDController.setI(0.00000005, pidSlotID);
    leftPIDController.setD(0, pidSlotID);
    leftPIDController.setFF(1/Constants.MotorConstants.NEO_MAX_VOLTAGE, pidSlotID);
    leftPIDController.setOutputRange(-1, 1, pidSlotID);

    leftPIDController.setP(0.0002, smartPIDSlotID);
    leftPIDController.setI(0.00000005, smartPIDSlotID);
    leftPIDController.setD(0.00003, smartPIDSlotID);
    leftPIDController.setFF(1/Constants.MotorConstants.NEO_MAX_VOLTAGE, smartPIDSlotID);
    leftPIDController.setOutputRange(-1, 1, smartPIDSlotID);

    leftPIDController.setFeedbackDevice(leftEncoder);


    leftMotor.setInverted(true);
    rightMotor.setInverted(true);

    //rightMotor.follow(leftMotor);

    limitSwitch = new DigitalInput(Constants.IntakeConstants.LIMIT_SWITCH_CHANNEL);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Encoder Velocity", leftEncoder.getVelocity());
    // SmartDashboard.putNumber("Encoder Position", leftEncoder.getPosition());
    // SmartDashboard.putBoolean("Encoder Target Reached; ", reachedTarget);

    if (leftEncoder.getVelocity() == -3000)
    reachedTarget = true;
    
    // System.out.println("Limit Switch: "+limitSwitch.get());
  }

  public void startIntake(double speed) {
    leftMotor.set(speed);
    // rightMotor.set(speed);
  }

  public void startIntakeReverse(double speed) {
    leftMotor.set(-speed);
    rightMotor.set(-speed);
  }

  public void stopIntake() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public double rampIntakeInput(double input, double target) {
    return (target - input) * Constants.IntakeConstants.RAMP_RATE;
  }

  public Command PIDControlSmartVelocity() {
  return new InstantCommand(() -> {/*leftEncoder.setPosition(0);*/ /*leftPIDController.setReference(leftEncoder.getPosition() + 100, ControlType.kPosition, pidSlotID);*/ leftPIDController.setReference(-3000, ControlType.kSmartVelocity, smartPIDSlotID); System.out.println("Reached velocity"); leftPIDController.setReference(leftEncoder.getPosition() + 100, ControlType.kPosition, pidSlotID); System.out.println("Reached position");}, this);
  }
}
