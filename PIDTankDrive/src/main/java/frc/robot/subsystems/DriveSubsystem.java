package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase{

    private final VictorSPX leftLeader = new VictorSPX(Constants.TankDriveConstants.FRONT_LEFT_MOTOR);
    private final VictorSPX leftFollower = new VictorSPX(Constants.TankDriveConstants.BACK_LEFT_MOTOR);
    private final VictorSPX rightLeader = new VictorSPX(Constants.TankDriveConstants.FRONT_RIGHT_MOTOR);
    private final VictorSPX rightFollower = new VictorSPX(Constants.TankDriveConstants.BACK_RIGHT_MOTOR);

    public DriveSubsystem() {

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        rightLeader.setInverted(true);
        rightFollower.setInverted(true);

    
    }

    public void arcadeDrive(double forward, double rotation) {
        double leftOutput = forward + rotation;
        double rightOutput = forward - rotation;

        leftLeader.set(ControlMode.PercentOutput, leftOutput);
        rightLeader.set(ControlMode.PercentOutput, rightOutput);
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler runs
    }
    
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
