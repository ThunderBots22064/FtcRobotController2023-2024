package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CONFIG.DRIVETRAIN;

public class DrivetrainSubsystem extends SubsystemBase {
    private final MecanumDrive drivetrain;
    private float speed;

    public DrivetrainSubsystem(HardwareMap hardMap) {
        final Motor mtFL, mtFR, mtBL, mtBR;

        mtFL = new Motor(hardMap, DRIVETRAIN.FRONT_LEFT_DEVICE);
        mtFR = new Motor(hardMap, DRIVETRAIN.FRONT_RIGHT_DEVICE);
        mtBL = new Motor(hardMap, DRIVETRAIN.BACK_LEFT_DEVICE);
        mtBR = new Motor(hardMap, DRIVETRAIN.BACK_RIGHT_DEVICE);

        mtFL.setInverted(DRIVETRAIN.FL_DIR_B);
        mtFR.setInverted(DRIVETRAIN.FR_DIR_B);
        mtBL.setInverted(DRIVETRAIN.BL_DIR_B);
        mtBR.setInverted(DRIVETRAIN.BR_DIR_B);

        mtFL.setRunMode(Motor.RunMode.RawPower);
        mtFR.setRunMode(Motor.RunMode.RawPower);
        mtBL.setRunMode(Motor.RunMode.RawPower);
        mtBR.setRunMode(Motor.RunMode.RawPower);
        
        drivetrain = new MecanumDrive(
            mtFL, mtFR,
            mtBL, mtBR
        );

        drivetrain.setRightSideInverted(false);

        speed = DRIVETRAIN.SPEED_DEF;
        drivetrain.setMaxSpeed(DRIVETRAIN.SPEED_MAX);
    }

    /**
     * Controls the robot relative to the robot itself
     * @param strafeSpeed the horizontal speed (+ = Right, - = Left)
     * @param forwardSpeed the vertical speed (+ = Forward, - = Backward)
     * @param turnSpeed the rotation speed (+ = Clockwise, - = Anti-Clockwise)
     * @param squareInputs whether the inputs should be squared to allow for finer controller
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean squareInputs) {
        drivetrain.driveRobotCentric(strafeSpeed * speed, forwardSpeed * speed, turnSpeed * speed, squareInputs);
    }

    /**
     * Controls the robot relative to the field
     * @param strafeSpeed the horizontal speed (+ = Right of controller, - = Left of controller)
     * @param forwardSpeed the vertical speed (+ = Away from controller, - = Towards controller)
     * @param turnSpeed the rotation speed (+ = Clockwise, - = Anti-Clockwise)
     * @param heading the heading IN DEGREES of the robot
     * @param squareInputs whether the inputs should be squared to allow for finer controller
     */
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading, boolean squareInputs) {
        drivetrain.driveFieldCentric(strafeSpeed * speed, forwardSpeed * speed, turnSpeed * speed, heading, squareInputs);
    }

    /**
     * Stops the robot
     */
    public void stop() {
        drivetrain.stop();
    }

    /**
     * Gets the robot speed multiplier
     * @return the robot's speed factor in decimal form
     */
    public double getSpeed() {
        return speed;
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
