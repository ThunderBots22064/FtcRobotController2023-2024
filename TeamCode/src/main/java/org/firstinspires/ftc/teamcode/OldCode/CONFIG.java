package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.hardware.*;

// This file contains all configuration stuff
public final class CONFIG {
    public final class CONTROLLER {
        // How far a joystick has to move before anything is detected
        public final static float STICK_DEADZONE = 0.40f;
        
        // How far a trigger has to move before anything is detected
        public final static float TRIGGER_DEADZONE = 0.50f;
        
        // The mode for driving (0 = Mecanum)
        public final static int DRIVE_MODE = 0;
    }
	
    public final class DRIVETRAIN {
        // The percent motor speed for the drivetrain
        public final static float SPEED_DEF = 0.50f;
        // The minimum speed of the drivetrain
        public final static float SPEED_MIN = 0.25f;
        // The maximum speed of the drivetrain
        public final static float SPEED_MAX = 0.80f;
        // The percent speed the robot changes when speed is adjusted
        public final static float SPEED_VAR = 0.10f;

        // Radius of the drive train wheels in mm
        public final static float WHEEL_RADIUS = 6.0f;
        
        /*--- PID Control for Drivetrain ---*/
        // K Proportional Coefficient
        public final static float Kp = 1.00f;
        // K Integral Coefficient
        public final static float Ki = 1.00f;
        // K Derivative Coefficient
        public final static float Kd = 1.00f;

        // String Values which correspond to the DriverHub's hardware map
        public final static String FRONT_RIGHT_DEVICE = "FRMotor";
        public final static String FRONT_LEFT_DEVICE = "FLMotor";
        public final static String BACK_RIGHT_DEVICE = "BRMotor";
        public final static String BACK_LEFT_DEVICE = "BLMotor";

        // Motor directions; 1 corresponds to forward while -1 corresponds to reversed
        public final static int FL_DIR = -1;
        public final static int FR_DIR = -1;
        public final static int BL_DIR =  1;
        public final static int BR_DIR = -1;
    }

    public final class CONTROL_SURFACES {
		public final class HOOK {
	        // The motor name for the hanging mechanism
	        public final static String HOOK_DEVICE = "thingy";
	        public final static float HOOK_SPEED_UP = 0.90f;
	        public final static float HOOK_SPEED_DOWN = 0.60f;
	        public final static int HOOK_FLOOR = 0;
	        public final static int HOOK_ROOF = 9200;
		}

		public final class DRONE {
	        // String value that corresponds to the servo 
			// device for the servo which launches the paper airplane
	        public final static String DRONE_DEVICE = "arm";
	        public final static int DRONE_DIR = 1;
		}
		
		public final class CLAW {
	        // Hardware Maps for Claw Pincer Servos
	        public final static String CLAW1_DEVICE = "Claw1";
	        public final static String CLAW2_DEVICE = "Claw2";
	        public final static int CLAW1_DIR = -1;
	        public final static int CLAW2_DIR = 1;
		}
        
		public final class ARM {
			// Hardware Map for the Arm
	        public final static String ARM_DEVICE = "Arm";
	        public final static int ARM_DIR = 1;
	        public final static float ARM_SPEED = 0.20f;
			
			/*--- PID Control for Arm ---*/
	        // K Proportional Coefficient
	        public final static float Kp = 0.15f;
	        // K Integral Coefficient
	        public final static float Ki = 0.05f;
	        // K Derivative Coefficient
	        public final static float Kd = 0.20f;
	        // Clamp for maximum value PID can use (+ and -)
	        public final static float CLAMP = 0.20f;
		}
    }
}
