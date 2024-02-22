package org.firstinspires.ftc.teamcode.CommandCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ImuSubsystem;

import java.lang.Math;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROLLER;

public class MoveRobot extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final ImuSubsystem imu;
    private final GamepadEx pad;

    /**
     * A command for controlling the robot drivetrain
     * @param pad the gamepad
     * @param drivetrain the drivetrain subsystem
     * @param imu the IMU subsystem
     */
    public MoveRobot(GamepadEx pad, DrivetrainSubsystem drivetrain, ImuSubsystem imu) {
        this.pad = pad;
        this.drivetrain = drivetrain;
        this.imu = imu;

        addRequirements(drivetrain);
        addRequirements(imu);
    }

    @Override
    public void execute() {
        double turn = 0.0;

        double leftTrigger = pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (leftTrigger >= CONTROLLER.TRIGGER_DEADZONE || rightTrigger >= CONTROLLER.TRIGGER_DEADZONE) {
            turn = leftTrigger > rightTrigger ? -drivetrain.getSpeed() : drivetrain.getSpeed();
        }
        
        double strafe = pad.getLeftX();
        if (Math.abs(strafe) < CONTROLLER.STICK_DEADZONE) {
            strafe = 0.0;
        }

        double forward = pad.getRightY();
        if (Math.abs(forward) < CONTROLLER.STICK_DEADZONE) {
            forward = 0.0;
        }

        drivetrain.driveRobotCentric(strafe, forward, turn, CONTROLLER.SQUARE_INPUTS);//, imu.getHeading());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
