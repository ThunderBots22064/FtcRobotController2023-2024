package org.firstinspires.ftc.teamcode.CommandCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ImuSubsystem;

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
        double turn = (pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) ? 1.0 : -1.0;
        
        double strafe = pad.getLeftX();
        strafe = strafe < CONTROLLER.STICK_DEADZONE ? 0 : strafe;

        double forward = pad.getRightY();
        forward = forward < CONTROLLER.STICK_DEADZONE ? 0 : forward;

        drivetrain.driveFieldCentric(strafe, forward, turn, imu.getHeading());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
