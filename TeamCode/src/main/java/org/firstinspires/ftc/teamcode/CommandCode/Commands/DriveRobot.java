package org.firstinspires.ftc.teamcode.CommandCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ImuSubsystem;

import java.lang.Math;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROLLER;

public class DriveRobot extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final ImuSubsystem imu;
    private final GamepadEx pad;

    private final DoubleSupplier horizontalCtrl;
    private final DoubleSupplier verticalCtrl;

    /**
     * A command for controlling the robot drivetrain
     * @param pad the gamepad
     * @param drivetrain the drivetrain subsystem
     * @param imu the IMU subsystem
     */
    public DriveRobot(GamepadEx pad, DrivetrainSubsystem drivetrain, ImuSubsystem imu) {
        this.pad = pad;
        this.drivetrain = drivetrain;
        this.imu = imu;

        horizontalCtrl = CONTROLLER.TEAGAN_MODE ? pad::getRightX : pad::getLeftX;
        verticalCtrl = CONTROLLER.TEAGAN_MODE ? () -> -pad.getLeftY() : pad::getRightY;

        addRequirements(drivetrain);
        addRequirements(imu);
    }

    @Override
    public void execute() {
        double turn = 0.0;
        double heading = pad.getButton(GamepadKeys.Button.B) ? 0.0 : imu.getHeading();

        double leftTrigger = pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (leftTrigger >= CONTROLLER.TRIGGER_DEADZONE || rightTrigger >= CONTROLLER.TRIGGER_DEADZONE) {
            turn = leftTrigger > rightTrigger ? -1.0 : 1.0;
        }
        
        double strafe = horizontalCtrl.getAsDouble();
        if (Math.abs(strafe) < CONTROLLER.STICK_DEADZONE) {
            strafe = 0.0;
        }

        double vertical = -verticalCtrl.getAsDouble();
        if (Math.abs(vertical) < CONTROLLER.STICK_DEADZONE) {
            vertical = 0.0;
        }

        drivetrain.driveFieldCentric(strafe, vertical, turn, heading, CONTROLLER.SQUARE_INPUTS);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
