// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PIDConstants;

public class PIDMotorSubsystem extends SubsystemBase {
  /** Creates a new PIDMotorSubsystem. */
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private SparkMaxPIDController pidController;

  private boolean targetPosReached = false;
  private boolean targetVelReached = false;
  private boolean shouldEnd = false;
  private boolean isPosCommand = false;
  private double allowedErr;
  private double targetPos;
  private double targetVel;

  /*private double[] PIDF;
  private double kP, kI, kD, kF;

  private boolean updated = false;*/

  public PIDMotorSubsystem() {

    motor = new CANSparkMax(Constants.PIDConstants.MOTOR_ID, MotorType.kBrushless);
    motor.restoreFactoryDefaults(); // TODO

    // leftMotor.restoreFactoryDefaults();
    encoder = motor.getEncoder();
    zeroEncoder();

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    pidController.setSmartMotionMaxVelocity(Constants.MotorConstants.NEO_MAX_VEL - 1000, PIDConstants.DEFAULT_SMART_ID);
    pidController.setSmartMotionMaxAccel(3500, PIDConstants.DEFAULT_SMART_ID);
    // pidController.setSmartMotionAllowedClosedLoopError(2, PIDConstants.DEFAULT_SMART_ID); TODO
    

    pidController.setP(0.00015, PIDConstants.DEFAULT_SMART_ID);
    pidController.setI(0.0, PIDConstants.DEFAULT_SMART_ID);
    pidController.setD(0.00003, PIDConstants.DEFAULT_SMART_ID);
    pidController.setFF(1.0/Constants.MotorConstants.NEO_MAX_VOLTAGE, PIDConstants.DEFAULT_SMART_ID);
    pidController.setOutputRange(-1, 1, PIDConstants.DEFAULT_SMART_ID);

    pidController.setP(0.0023, PIDConstants.DEFAULT_PID_ID);
    pidController.setI(0.00000005, PIDConstants.DEFAULT_PID_ID);
    pidController.setD(0, PIDConstants.DEFAULT_PID_ID);
    pidController.setFF(1.0/Constants.MotorConstants.NEO_MAX_VOLTAGE, PIDConstants.DEFAULT_PID_ID);
    pidController.setOutputRange(-1, 1, PIDConstants.DEFAULT_PID_ID);

    /*PIDF = getPIDF(PIDConstants.DEFAULT_SMART_ID);

    SmartDashboard.putNumber("kP", PIDF[0]);
    SmartDashboard.putNumber("kI", PIDF[1]);
    SmartDashboard.putNumber("kD", PIDF[2]);
    SmartDashboard.putNumber("kF", PIDF[3]);*/

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (encoder.getVelocity() == targetVel && !isPosCommand) {targetVelReached = true;}
    else {targetVelReached = false;}

    if (encoder.getPosition() == targetPos && isPosCommand) {targetPosReached = true;}
    else {targetPosReached = false;}

    if ((Math.abs(targetVel - encoder.getVelocity()) < allowedErr && !isPosCommand) || (Math.abs(targetPos - encoder.getPosition()) < allowedErr && isPosCommand)) {
      shouldEnd = true;
    }

    else {shouldEnd = false;}
    
    
    SmartDashboard.putNumber("Encoder Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
    SmartDashboard.putBoolean("Target Velocity Reached", targetVelReached);
    SmartDashboard.putBoolean("Target Position Reached", targetPosReached);
    SmartDashboard.putBoolean("Command should end", shouldEnd);
    SmartDashboard.putNumber("Allowed Error", pidController.getSmartMotionAllowedClosedLoopError(PIDConstants.DEFAULT_SMART_ID));
    SmartDashboard.putNumber("ErrorVel", targetVel - encoder.getVelocity());
    SmartDashboard.putNumber("ErrorPos", targetPos - encoder.getPosition());

    /*kP = SmartDashboard.getNumber("kP", PIDF[0]);
    kI = SmartDashboard.getNumber("kI", PIDF[1]);
    kD = SmartDashboard.getNumber("kD", PIDF[2]);
    kF = SmartDashboard.getNumber("kF", PIDF[3]);

    if (updated) {
      SmartDashboard.putData("Update", new InstantCommand(() -> {if (!updated) {
                                                                    setPIDF(kP, kI, kD, kF, PIDConstants.DEFAULT_SMART_ID); System.out.println("Updated");
                                                                    updated = true;
                                                                  }
                                                                }, this));
    }

    else {
      SmartDashboard.putData("Update", new InstantCommand());
    }*/
  }

  public void setEncoderPosition(double position) {
    encoder.setPosition(position);
  }

  public void zeroEncoder() {
    encoder.setPosition(0);
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public double getEncoderVelocity() {
    return encoder.getVelocity();
  }

  public CANSparkMax getMotor() {
    return motor;
  }

  public void PIDPosition(double position) {
    targetPosReached = false;
    isPosCommand = true;
    targetPos = position;
    allowedErr = targetPos * PIDConstants.ALLOWED_ERROR;
    pidController.setReference(position, ControlType.kPosition, Constants.PIDConstants.DEFAULT_PID_ID);
  }

  public void PIDVelocity(double vel) {
    targetVelReached = false;
    isPosCommand = false;
    targetVel = vel;
    allowedErr = targetVel * PIDConstants.ALLOWED_ERROR;
    pidController.setReference(vel, ControlType.kVelocity, Constants.PIDConstants.DEFAULT_PID_ID);
  }

  public void PIDSmartPosition(double pos) {
    targetPosReached = false;
    isPosCommand = true;
    targetPos = pos;
    allowedErr = targetPos * PIDConstants.ALLOWED_ERROR;
    pidController.setReference(pos, ControlType.kSmartMotion, Constants.PIDConstants.DEFAULT_SMART_ID);
  }

  public void PIDSmartVelocity(double vel)  {
    targetVelReached = false;
    isPosCommand = false;
    targetVel = vel;
    allowedErr = targetVel * PIDConstants.ALLOWED_ERROR;
    pidController.setReference(vel, ControlType.kSmartVelocity, PIDConstants.DEFAULT_SMART_ID);
  }

  public void setPositionConversionFactor(double factor) {
    encoder.setPositionConversionFactor(factor);
  }

  public void setVelocityConversionFactor(double factor) {
    encoder.setVelocityConversionFactor(factor);
  }

  public void setSmartMotionMaxVelocity(double max_velocity) {
    pidController.setSmartMotionMaxVelocity(max_velocity, Constants.PIDConstants.DEFAULT_SMART_ID);
  }

  public void setSmartMotionMaxVelocity(double max_velocity, int slotID) {
    pidController.setSmartMotionMaxVelocity(max_velocity, slotID);
  }

  public void setSmartMotionMaxAccel(double max_accel) {
    pidController.setSmartMotionMaxAccel(max_accel, Constants.PIDConstants.DEFAULT_SMART_ID);
  }

  public void setSmartMotionMaxAccel(double max_accel, int slotID) {
    pidController.setSmartMotionMaxAccel(max_accel, slotID);
  }

  public void setSmartMotionAllowedClosedLoopError(double error) {
    pidController.setSmartMotionAllowedClosedLoopError(error, Constants.PIDConstants.DEFAULT_SMART_ID);
  }

  public void setSmartMotionAllowedClosedLoopError(double error, int slotID) {
    pidController.setSmartMotionAllowedClosedLoopError(error, slotID);
  }

  public double getSmartMotionAllowedClosedLoopError(int slotID) {
    return pidController.getSmartMotionAllowedClosedLoopError(slotID);
  }

  public void setPIDF(double kP, double kI, double kD, double kF, int slotID) {
    pidController.setP(kP, slotID);
    pidController.setI(kI, slotID);
    pidController.setP(kP, slotID);
    pidController.setFF(kF, slotID);
    pidController.setOutputRange(-1, 1, slotID);
  }

  public double[] getPIDF(int slotID) {
    double[] PIDF = {pidController.getP(slotID), pidController.getI(slotID), pidController.getD(slotID), pidController.getFF(slotID)};
    return PIDF;
  }

}
