package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CONFIG.DRIVETRAIN;

public class DrivetrainSubsystem extends SubsystemBase {
    private final MecanumDrive drivetrain;
    private final Motor mtFL, mtFR, mtBL, mtBR;
    private float speed;

    public DrivetrainSubsystem(HardwareMap hardMap) {
        mtFL = new Motor(hardMap, DRIVETRAIN.FRONT_LEFT_DEVICE);
        mtFR = new Motor(hardMap, DRIVETRAIN.FRONT_RIGHT_DEVICE);
        mtBL = new Motor(hardMap, DRIVETRAIN.BACK_LEFT_DEVICE);
        mtBR = new Motor(hardMap, DRIVETRAIN.BACK_RIGHT_DEVICE);

        mtFL.setInverted(DRIVETRAIN.FL_DIR_B);
        mtFR.setInverted(DRIVETRAIN.FR_DIR_B);
        mtBL.setInverted(DRIVETRAIN.BL_DIR_B);
        mtBR.setInverted(DRIVETRAIN.BR_DIR_B);
        
        drivetrain = new MecanumDrive(
            mtFL, 
            mtFR,
            mtBL, 
            mtBR
        );

        speed = DRIVETRAIN.SPEED_DEF;
        drivetrain.setMaxSpeed(DRIVETRAIN.SPEED_MAX);
    }

    /**
     * Controls the robot relative to the robot itself
     * @param strafeSpeed the horizontal speed (+ = Right, - = Left)
     * @param forwardSpeed the vertical speed (+ = Forward, - = Backward)
     * @param turnSpeed the rotation speed (+ = Clockwise, - = Anti-Clockwise)
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drivetrain.driveRobotCentric(strafeSpeed * speed, forwardSpeed * speed, turnSpeed * speed);
    }

    /**
     * Controls the robot relative to the field
     * @param strafeSpeed the horizontal speed (+ = Right of controller, - = Left of controller)
     * @param forwardSpeed the vertical speed (+ = Away from controller, - = Towards controller)
     * @param turnSpeed the rotation speed (+ = Clockwise, - = Anti-Clockwise)
     * @param heading the heading IN DEGREES of the robot
     */
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading) {
        drivetrain.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
    }

    /**
     * Stops the robot
     */
    public void stop() {
        drivetrain.stop();
    }

    /**
     * Increases the percent speed of the robot given that it wont exceed the limit speed
     */
    public void increaseSpeed() {
        if ((speed + DRIVETRAIN.SPEED_VAR) <= DRIVETRAIN.SPEED_MAX) {
            speed += DRIVETRAIN.SPEED_VAR;
        }
    }

    /**
     * Decreases the percent speed of the robot given that it wont exceed the minimum speed
     */
    public void decreaseSpeed() {
        if ((speed - DRIVETRAIN.SPEED_VAR) >= DRIVETRAIN.SPEED_MIN) {
            speed -= DRIVETRAIN.SPEED_VAR;
        }
    }
}
