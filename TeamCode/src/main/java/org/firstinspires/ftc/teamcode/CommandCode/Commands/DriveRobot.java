package org.firstinspires.ftc.teamcode.CommandCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ImuSubsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.Math;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROLLER;

public class DriveRobot extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final ImuSubsystem imu;
    private final GamepadEx pad;

    Telemetry telemetry;

    /**
     * A command for controlling the robot drivetrain
     * @param pad the gamepad
     * @param drivetrain the drivetrain subsystem
     * @param imu the IMU subsystem
     */
    public DriveRobot(GamepadEx pad, DrivetrainSubsystem drivetrain, ImuSubsystem imu, Telemetry telemetry) {
        this.pad = pad;
        this.drivetrain = drivetrain;
        this.imu = imu;

        this.telemetry = telemetry;

        addRequirements(drivetrain);
        addRequirements(imu);
    }

    @Override
    public void execute() {
        double turn = 0.0;

        double leftTrigger = pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (leftTrigger >= CONTROLLER.TRIGGER_DEADZONE || rightTrigger >= CONTROLLER.TRIGGER_DEADZONE) {
            turn = leftTrigger > rightTrigger ? -1.0 : 1.0;
        }
        
        double strafe = pad.getLeftX();
        if (Math.abs(strafe) < CONTROLLER.STICK_DEADZONE) {
            strafe = 0.0;
        }

        double forward = -pad.getRightY();
        if (Math.abs(forward) < CONTROLLER.STICK_DEADZONE) {
            forward = 0.0;
        }

        telemetry.addData("IMU FROM MOVE: ", imu.getHeading());
        telemetry.addData("RAW IMU FROM MOVE: ", imu.getHeadingRaw());
        telemetry.update();

        if (pad.getButton(GamepadKeys.Button.B)) {
            drivetrain.driveRobotCentric(strafe, forward, turn, CONTROLLER.SQUARE_INPUTS);
        } else {
            drivetrain.driveFieldCentric(strafe, forward, turn, imu.getHeading(), CONTROLLER.SQUARE_INPUTS);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
