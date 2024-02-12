package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CONFIG.DRIVETRAIN;
public class DrivetrainSubsystem extends SubsystemBase {
    private final MecanumDrive chassis;

    public DrivetrainSubsystem(HardwareMap hardMap) {
        chassis = new MecanumDrive(
                new Motor(hardMap, DRIVETRAIN.FRONT_LEFT_DEVICE), new Motor(hardMap, DRIVETRAIN.FRONT_RIGHT_DEVICE),
                new Motor(hardMap, DRIVETRAIN.BACK_LEFT_DEVICE), new Motor(hardMap, DRIVETRAIN.BACK_RIGHT_DEVICE)
        );
    }

    /**
     * Controls the robot relative to the robot itself
     * @param strafeSpeed the horizontal speed
     * @param forwardSpeed the vertical speed
     * @param turnSpeed the rotation speed
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        chassis.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
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
}
