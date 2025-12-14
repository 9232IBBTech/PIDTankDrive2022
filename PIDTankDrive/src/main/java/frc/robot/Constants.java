// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
      }

      public static class MotorConstants {
        public static final int NEO_MAX_VEL = 5676;
        public static final int NEO_MAX_VOLTAGE = 473;
      }
    
      public static class TankDriveConstants {
        public static final int FRONT_LEFT_MOTOR = 3;
        public static final int BACK_LEFT_MOTOR = 4;
        public static final int FRONT_RIGHT_MOTOR = 1;
        public static final int BACK_RIGHT_MOTOR = 2;
        public static final double ChasisSpeedFactor = 0.7;
        public static final double rampRate = 0.07;
      }
    
      public static class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double JOYSTICK_DEADZONE = 0.05;
        public static final XboxController controller = new XboxController(DRIVER_CONTROLLER_PORT);
        public static final JoystickButton ButtonLB = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        public static final JoystickButton ButtonRB = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
        public static final Trigger xButton = new JoystickButton(controller, XboxController.Button.kX.value);
        public static final Trigger bButton = new JoystickButton(controller, XboxController.Button.kB.value);
        public static final Trigger aButton = new JoystickButton(controller, XboxController.Button.kA.value);
        public static Trigger leftTrigger = new Trigger(() -> controller.getLeftTriggerAxis() > 0.3);
        public static Trigger rightTrigger = new Trigger(() -> controller.getRightTriggerAxis() > 0.3);
        
      }

      public static class IntakeConstants {
        public static final int INTAKE_LEFT_MOTOR = 61;
        public static final int INTAKE_RIGHT_MOTOR = 41;
        public static final int LIMIT_SWITCH_CHANNEL = 5;
        public static final double INTAKE_SPEED = 0.5;
        public static final double OUTTAKE_SPEED = 0.5;
        public static final double RAMP_RATE = 0.3;
        public static final double GEAR_RATIO = 1/9;
      }

      public static class PIDConstants {
        public static final int MOTOR_ID = 60;
        public static final int DEFAULT_SMART_ID = 1;
        public static final int DEFAULT_PID_ID = 0;
        public static final double ALLOWED_ERROR = 0.02;

      }

      
    }
