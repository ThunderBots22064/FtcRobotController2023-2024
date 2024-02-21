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
    private final Teleemtry telemetry;
    
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
    }
}