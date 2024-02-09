package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.CONFIG;

import java.lang.Math;

@TeleOp
public class OldTeleOp extends OpMode {
    /*** CONFIG PRESETS ***/
    float SPEED_DEF = CONFIG.DRIVETRAIN.SPEED_DEF;
    float SPEED_MIN = CONFIG.DRIVETRAIN.SPEED_MIN;
    float SPEED_MAX = CONFIG.DRIVETRAIN.SPEED_MAX;
    float SPEED_VAR = CONFIG.DRIVETRAIN.SPEED_VAR;
    float SPEED_HOL = SPEED_DEF;
    
    float STICK_DEADZONE = CONFIG.CONTROLLER.STICK_DEADZONE;
    float TRIGGER_DEADZONE = CONFIG.CONTROLLER.TRIGGER_DEADZONE;
    
    float HOOK_SPEED_UP = CONFIG.CONTROL_SURFACES.HOOK.HOOK_SPEED_UP;
    float HOOK_SPEED_DOWN = CONFIG.CONTROL_SURFACES.HOOK.HOOK_SPEED_DOWN;
    float ARM_SPEED = CONFIG.CONTROL_SURFACES.ARM.ARM_SPEED;

    // Power Matrixes for driving
    public final static float[][] DRIVE_ARRAY = {
        // Mecanum Driving
        // Left-Stick Up
            {1f, 1f, 
            1f, 1f},

            // Left-Stick Down
            {-1f, -1f, 
            -1f, -1f},

            // Right-Stick Right
            {1f, -1f, 
            -1f, 1f},

            // Right-Stick Left
            {-1f, 1f, 
            1f, -1f},

            // Right Trigger
            {1f, -1f, 
            1f, -1f},

            // Left Trigger
            {-1f, 1f, 
            -1f, 1f}
    };

    // To make the joysticks easier to understand there are 4 boxes placed on each joystick (Imaginary)
    // If the joystick is in the center the box is 0, right or up is 1, and left or down is -1
    // l and r corresponds to left and right joysticks
    // The X and Y are the axis that the boxes are on
    int lXBox = 0;
    int lYBox = 0;
    int rXBox = 0;
    int rYBox = 0;
    
    // Stick on 2nd Controller
    int lYBox2 = 0;

    // Multipliers to prevent drift
    float leftApply = 1;
    float rightApply = 1;
    float powMatMax = 1;
    
    // Control Triggers - These allow detection of single button presses 
    boolean BUMPER_LEFT = false;
    boolean BUMPER_RIGHT = false;
    boolean HOOK_ACTIVE = false;

    // Array which indicates if a wheel will be powered on this tick (Power Matrix)
    // In this order FL, FR, BL, BR
    float[] powMat = {0f, 0f, 
                      0f, 0f};
    // Used to detect if the final powMat changed since the last tick
    float[] lastPowMat = {0f, 0f, 
                          0f, 0f};
    
    // setArr()
    
    // An encoder offset that is subtracted from every loop to allow quick encoder resets
    // when powMat has changed
    int[] encoderOffset = {0, 0, 
                           0, 0};

    // Similar to powMat pidMat is a matrix that controls PID control for error correction
    float[] pidMat = {0f, 0f, 
                      0f, 0f};

    // For PID to work on Mecanum wheels it must know which wheels are supposed to have equal values so it uses wheel pairs to detect this for every move.
    int[][] wheelPair = {{0, 0}, 
                           {0, 0}};

    // Drivetrain motors
    DcMotor mtFL = null; // Front Left
    DcMotor mtFR = null; // Front Right
    DcMotor mtBL = null; // Back Left
    DcMotor mtBR = null; // Back Right
    
    /*** CONTROL SURFACES ***/
    // Hook Motor
    DcMotor mtHook = null;
    // Drone Servo
    CRServo svDrone = null;
    
    // Arm Motor and Claw Servos
    CRServo svClaw1 = null;
    CRServo svClaw2 = null;
    DcMotor mtArm1 = null;
    DcMotor mtArm2 = null;
    int updateTick = 0;
    int lastArmPos = 0;
    int expecArmPos = 0;
    float armProp = 0;
    float armDeriv = 0;
    float armInteg = 0;
    
    public void init() {
    // Setup motors
      // Motors are first identified with the 'mt' (Motor)
      // They're then identified with F (Front) or B (Back)
      // Next the side is identified with L (Left) or R (Right)  
        mtFL = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.FRONT_LEFT_DEVICE);
        mtFR = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.FRONT_RIGHT_DEVICE);
        mtBL = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.BACK_LEFT_DEVICE);
        mtBR = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.BACK_RIGHT_DEVICE);
        
        mtFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        mtFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        mtFL.setDirection(CONFIG.DRIVETRAIN.FL_DIR);
        mtFR.setDirection(CONFIG.DRIVETRAIN.FR_DIR);
        mtBL.setDirection(CONFIG.DRIVETRAIN.BL_DIR);
        mtBR.setDirection(CONFIG.DRIVETRAIN.BR_DIR);

        // Drone Servo
        svDrone = hardwareMap.get(CRServo.class, CONFIG.CONTROL_SURFACES.DRONE.DRONE_DEVICE);
        svDrone.setDirection(CONFIG.CONTROL_SURFACES.DRONE.DRONE_DIR);
        
        // Hook Motor
        mtHook = hardwareMap.get(DcMotor.class, CONFIG.CONTROL_SURFACES.HOOK.HOOK_DEVICE);
        // Reset Hook Encoder
        mtHook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtHook.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Arm Motors
        mtArm1 = hardwareMap.get(DcMotor.class, CONFIG.CONTROL_SURFACES.ARM.ARM1_DEVICE);
        mtArm1.setDirection(CONFIG.CONTROL_SURFACES.ARM.ARM1_DIR);
        mtArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtArm2 = hardwareMap.get(DcMotor.class, CONFIG.CONTROL_SURFACES.ARM.ARM2_DEVICE);
        mtArm2.setDirection(CONFIG.CONTROL_SURFACES.ARM.ARM2_DIR);
        mtArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
		// Claw Servos
        svClaw1 = hardwareMap.get(CRServo.class, CONFIG.CONTROL_SURFACES.CLAW.CLAW1_DEVICE);
        svClaw2 = hardwareMap.get(CRServo.class, CONFIG.CONTROL_SURFACES.CLAW.CLAW2_DEVICE);
        svClaw1.setDirection(CONFIG.CONTROL_SURFACES.CLAW.CLAW1_DIR);
        svClaw2.setDirection(CONFIG.CONTROL_SURFACES.CLAW.CLAW2_DIR);
    }

    public void loop() {
        // Define Things
        SPEED_DEF = SPEED_HOL;
        
        powMat = new float[] {0f, 0f,
                              0f, 0f};
        leftApply = 1;
        rightApply = 1;

        // Y-values are inverted on gamepad as up returns negative values
        lXBox = calcBox(gamepad1.left_stick_x, STICK_DEADZONE);
        lYBox = calcBox(-gamepad1.left_stick_y, STICK_DEADZONE);
        rXBox = calcBox(gamepad1.right_stick_x, STICK_DEADZONE);
        rYBox = calcBox(-gamepad1.right_stick_y, STICK_DEADZONE);
        
        // Y value for Arm control
        lYBox2 = calcBox(-gamepad2.left_stick_y, STICK_DEADZONE);

/***
------GAMEPAD2 CONTROLS------
***/
        // If the gamepad's x button is pressed launch the plane
        if (gamepad2.x) {
            svDrone.setPower( 1 );
        } else {
            svDrone.setPower( 0 );
        }
        
        // Mark of Josh
        //hacked >:)
        
        // Controls Servo Pincers
        // Right trigger closes
        if (gamepad2.right_trigger > TRIGGER_DEADZONE) {
            svClaw1.setPower(1);
            svClaw2.setPower(1);
        }
        // Left trigger opens
        else if (gamepad2.left_trigger > TRIGGER_DEADZONE) {
            svClaw1.setPower(-1);
            svClaw2.setPower(-1);
        } else {
            svClaw1.setPower(0);
            svClaw2.setPower(0);
        }
        
        // Use gamepad2 left joystick y value to control arm
        /*if (lYBox2 != 0) {
            mtArm.setPower(-gamepad2.left_stick_y * CONFIG.CONTROL_SURFACES.ARM_SPEED);
        } else {
            mtArm.setPower(0);
        }*/

        // COMMMENT THIS OUT IN AN EMERGENCY
        // /*
        int armPos = (mtArm1.getCurrentPosition() + mtArm2.getCurrentPosition()) / 2;
        if (lYBox2 != 0) {
            mtArm1.setPower(-gamepad2.left_stick_y * ARM_SPEED);
            mtArm2.setPower(-gamepad2.left_stick_y * ARM_SPEED);
            expecArmPos = armPos;
            updateTick = 0;
        } else {
            if (!gamepad2.b && updateTick > 30) {
                int error = expecArmPos - armPos;
                armProp = error * CONFIG.CONTROL_SURFACES.ARM.Kp;
                armInteg += error * CONFIG.CONTROL_SURFACES.ARM.Ki;
                armDeriv = (lastArmPos - armPos) * CONFIG.CONTROL_SURFACES.ARM.Kd;
                float total = armProp + armInteg + armDeriv;
                total *= CONFIG.CONTROL_SURFACES.ARM.CLAMP;
                if (Math.abs(total) > CONFIG.CONTROL_SURFACES.ARM.CLAMP) {
                    if (total < 0) {    
                        total = -CONFIG.CONTROL_SURFACES.ARM.CLAMP;
                    }
                    total = CONFIG.CONTROL_SURFACES.ARM.CLAMP;
                }
                mtArm1.setPower(total);
                mtArm2.setPower(total);
                updateTick = 0;
            } else if (gamepad2.b) {
                mtArm1.setPower(0);
                mtArm2.setPower(0);
            }
        }
        // */
        lastArmPos = armPos;
        
        // Y Moves hook up
        if (gamepad2.y) {
            // COMMENT THE IF STATEMENT TO DISABLE ENCODER SAFETY ON THE HOOOK
            // if (mtHook.getCurrentPosition() < CONFIG.CONTROL_SURFACES.HOOK_ROOF || (gamepad2.left_bumper && gamepad2.right_bumper)) {
                mtHook.setPower(HOOK_SPEED_UP);
                HOOK_ACTIVE = true;
            // }
        } 
        // A Moves hook down
        else if (gamepad2.a) {
            // COMMENT THE IF STATEMENT TO DISABLE ENCODER SAFETY ON THE HOOOK
            // if (mtHook.getCurrentPosition() > CONFIG.CONTROL_SURFACES.HOOK_FLOOR ||  (gamepad2.left_bumper && gamepad2.right_bumper)) {
                mtHook.setPower(-HOOK_SPEED_DOWN);
                HOOK_ACTIVE = true;
            // }
        }
        // Disable hook motors when not in use
        else if (HOOK_ACTIVE) {
            mtHook.setPower(0);
            HOOK_ACTIVE = false;
        }
      

        
/***
------GAMEPAD1 CONTROLS------
***/
        // Left joystick controls horizontal strafing movement
        if (lYBox != 0) {
            if (lYBox == 1) {
                powMat = addMat(powMat, DRIVE_ARRAY[0]);
            }
            else if (lYBox == -1) {
                powMat = addMat(powMat, DRIVE_ARRAY[1]);
            }
        } 
        // Right joystick controls vertical forward/back movement
        if (rXBox != 0) {
            if (rXBox == 1) {
                powMat = addMat(powMat, DRIVE_ARRAY[2]);
            }
            else if (rXBox == -1) {
                powMat = addMat(powMat, DRIVE_ARRAY[3]);
            }
        }

        // Right Trigger turns clockwise
        if (gamepad1.right_trigger > TRIGGER_DEADZONE) {
            powMat = addMat(powMat, DRIVE_ARRAY[4]);
        }
        // Left Trigger turns counterclockwise
        else if (gamepad1.left_trigger > TRIGGER_DEADZONE) {
            powMat = addMat(powMat, DRIVE_ARRAY[5]);
        }

        // Left-Bumper Decreases Speed by SPEED_VAR
        if (gamepad1.left_bumper && !BUMPER_LEFT) {
            if (SPEED_HOL - SPEED_VAR >= SPEED_MIN) {
                SPEED_HOL -= SPEED_VAR;
            }
            BUMPER_LEFT = true;
        } 
        else if (!gamepad1.left_bumper) {
            BUMPER_LEFT = false;
        }

        // Right-Bumper Increases the Speed by SPEED_VAR
        if (gamepad1.right_bumper && !BUMPER_RIGHT) {
            if (SPEED_HOL + SPEED_VAR <= SPEED_MAX) {
                SPEED_HOL += SPEED_VAR;
            }
            BUMPER_RIGHT = true;
        } else if (!gamepad1.right_bumper) {
            BUMPER_RIGHT = false;
        }
        
        // telemetry.addData("Hook Encoder: ", mtHook.getCurrentPosition());

        /*
        telemetry.addData("Motor Arm", mtArm.getCurrentPosition());
        
        telemetry.addData("Motor Front Left", powMat[0]);
        telemetry.addData("Motor Front Right", powMat[1]);
        telemetry.addData("Motor Back Left", powMat[2]);
        telemetry.addData("Motor Back Right", powMat[3]);
        
        telemetry.addData("Encoder Front Left", pos[0]);
        telemetry.addData("Encoder Front Right", pos[1]);
        telemetry.addData("Encoder Back Left", pos[2]);
        telemetry.addData("Encoder Back Right", pos[3]);

        telemetry.addData("Error (R-L)", error);
        telemetry.addData("Proportion", prop);
        

        telemetry.addData("Right Stick X: ", gamepad1.right_stick_x);
        telemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);

        telemetry.addData("lYBox: ", lYBox);
        telemetry.addData("rXBox: ", rXBox);*/
        
        
        for (int i = 0; i < powMat.length; i++) {
            if (Math.abs(powMat[i]) > powMatMax) {
                powMatMax = Math.abs(powMat[i]);
            }
        }
        for (int i = 0; i < powMat.length; i++) {
            powMat[i] /= powMatMax;
        }


        /* PID CONTROLLER */
        
        int[] pos = {mtFL.getCurrentPosition(), mtFR.getCurrentPosition(),
                     mtBL.getCurrentPosition(), mtBR.getCurrentPosition()};

        boolean swap = false;
        for (int i = 0; i < powMat.length; i++) {
            if (powMat[i] != lastPowMat[i]) {
                swap = true;
                break;
            }
        }

        if (swap) {
            setArr(encoderOffset, pos);
        }
        
        /*for (int i = 0; i < pos.length; i++) {
            telemetry.addData("Encoder Value: ", pos[i]);
        }*/
        

        mtFL.setPower(powMat[0] * SPEED_DEF);
        mtFR.setPower(powMat[1] * SPEED_DEF);
        mtBL.setPower(powMat[2] * SPEED_DEF);
        mtBR.setPower(powMat[3] * SPEED_DEF);    

        // telemetry.update();
        updateTick++;
    }

    // Returns an array where each value of matrixB is added to matrixA
    public static float[] addMat(float[] matrixA, float[] matrixB) {
        float[] matrix = new float[matrixA.length];
        
        for (int i = 0; i < matrixA.length; i++) {
            matrix[i] = matrixA[i] + matrixB[i];
        }
        
        return matrix;
    }
   
    public static int calcBox(float position, float deadzone) {
        if (Math.abs(position) > deadzone) {
            if (position < 0) {
                return -1;
            }
            return 1;
        }
        return 0;
    }
    
    public static void setArr(float[] set, float[] get) {
        for (int i = 0; i < set.length; i++) {
            set[i] = get[i];
        }
    }
    
    public static void setArr(int[] set, int[] get) {
        for (int i = 0; i < set.length; i++) {
            set[i] = get[i];
        }
    }
}