package org.firstinspires.ftc.teamcode.CommandCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ImuSubsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROLLER;

public class TestMove extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final ImuSubsystem imu;
    private final GamepadEx pad;
    private final Telemetry telemetry;
    
    /**
     * Creates a new TestMove command that tests the drivetrain
     * @param pad the gamepad
     * @param drivetrain the drivetrain subsystem
     * @param imu the IMU subsystem
     * @param telemetry the telemetry object through which logs can be sent
     */
    public TestMove(GamepadEx pad, DrivetrainSubsystem drivetrain, ImuSubsystem imu, Telemetry telemetry) {
        this.drivetrain = drivetrain;
        this.imu = imu;
        this.pad = pad;
        this.telemetry = telemetry;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double strafe = pad.getLeftX();
        double vertical = pad.getLeftY();
        double turn = pad.getRightX();
        
        telemetry.addData("Strafe Value: ", strafe);
        telemetry.addData("Vertical Value: ", vertical);
        telemetry.addData("Turn Value: ", turn);

        telemetry.addData("Right Y: ", pad.getRightY());

        telemetry.addData("Speed: ", drivetrain.getSpeed());

        telemetry.addData("IMU Heading Raw: ", imu.getHeadingRaw());
        telemetry.addData("IMU Heading Scaled: ", imu.getHeading());

        drivetrain.driveRobotCentric(
            pad.getLeftX(),
            pad.getLeftY(),
            pad.getRightX(),
            false
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}