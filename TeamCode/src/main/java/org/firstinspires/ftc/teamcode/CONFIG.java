package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

// This file contains all configuration stuff
public final class CONFIG {
    public final static class CONTROLLER {
        // How far a joystick has to move before anything is detected
        public final static float STICK_DEADZONE = 0.40f;
        
        // How far a trigger has to move before anything is detected
        public final static float TRIGGER_DEADZONE = 0.50f;
    }
	
    public final static class DRIVETRAIN {
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

        // Motor directions
        public final static DcMotorSimple.Direction FL_DIR = DcMotorSimple.Direction.REVERSE;
        public final static DcMotorSimple.Direction FR_DIR = DcMotorSimple.Direction.REVERSE;
        public final static DcMotorSimple.Direction BL_DIR = DcMotorSimple.Direction.FORWARD;
        public final static DcMotorSimple.Direction BR_DIR = DcMotorSimple.Direction.REVERSE;

        // Motor directions in boolean form (True -> Reverse, False -> Forward)
        public final static boolean FL_DIR_B = true;
        public final static boolean FR_DIR_B = true;
        public final static boolean BL_DIR_B = false;
        public final static boolean BR_DIR_B = true;
    }

    public final static class CONTROL_SURFACES {
        public final static class INTERTIALMU {
            public final static String IMU_DEVICE = "imu";
        }
		public final static class HOOK {
	        // The motor name for the hanging mechanism
	        public final static String HOOK_DEVICE = "thingy";
	        public final static float HOOK_SPEED_UP = 0.90f;
	        public final static float HOOK_SPEED_DOWN = 0.60f;

            public final static DcMotorSimple.Direction HOOK_DIR = DcMotorSimple.Direction.FORWARD;
	        public final static int HOOK_FLOOR = 0;
	        public final static int HOOK_ROOF = 9200;
		}

		public final static class DRONE {
	        // String value that corresponds to the servo 
			// device for the servo which launches the paper airplane
	        public final static String DRONE_DEVICE = "arm";
	        public final static DcMotorSimple.Direction DRONE_DIR = DcMotorSimple.Direction.REVERSE;
		}
		
		public final static class CLAW {
	        // Hardware Maps for Claw Pincer Servos
	        public final static String CLAW1_DEVICE = "Claw1";
	        public final static String CLAW2_DEVICE = "Claw2";
	        public final static DcMotorSimple.Direction CLAW1_DIR = DcMotorSimple.Direction.REVERSE;
	        public final static DcMotorSimple.Direction CLAW2_DIR = DcMotorSimple.Direction.FORWARD;
		}
        
		public final static class ARM {
			// Hardware Map for the Arm
	        public final static String ARM1_DEVICE = "Arm1";
            public final static String ARM2_DEVICE = "Arm2";
	        public final static DcMotorSimple.Direction ARM1_DIR = DcMotorSimple.Direction.FORWARD;
            public final static DcMotorSimple.Direction ARM2_DIR = DcMotorSimple.Direction.REVERSE;
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

            // The number of ticks between corrections, control hub runs at about 60 Hz
            public final static int TICKS = 30;
		}
    }
}