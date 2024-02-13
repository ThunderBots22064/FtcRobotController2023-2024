package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CONFIG.DRIVETRAIN;
public class DrivetrainSubsystem extends SubsystemBase {
    private final MecanumDrive chassis;
    private float speed;

    public DrivetrainSubsystem(HardwareMap hardMap) {
        chassis = new MecanumDrive(
                new Motor(hardMap, DRIVETRAIN.FRONT_LEFT_DEVICE), new Motor(hardMap, DRIVETRAIN.FRONT_RIGHT_DEVICE),
                new Motor(hardMap, DRIVETRAIN.BACK_LEFT_DEVICE), new Motor(hardMap, DRIVETRAIN.BACK_RIGHT_DEVICE)
        );

        speed = DRIVETRAIN.SPEED_DEF;
    }

    /**
     * Controls the robot relative to the robot itself
     * @param strafeSpeed the horizontal speed
     * @param forwardSpeed the vertical speed
     * @param turnSpeed the rotation speed
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        chassis.driveRobotCentric(strafeSpeed * speed, forwardSpeed * speed, turnSpeed * speed);
    }

    /**
     * Controls the robot relative to the field
     * @param strafeSpeed the horizontal speed
     * @param forwardSpeed the vertical speed
     * @param turnSpeed the rotation speed
     * @param heading the heading IN DEGREES of the robot
     */
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading) {
        chassis.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
    }

    /**
     * Stops the robot
     */
    public void stop() {
        chassis.stop();
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
