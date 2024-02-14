package org.firstinspires.ftc.teamcode.CommandCode;

import com.arcrobotics.ftclib.command.button.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;

import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.CONFIG;
import org.firstinspires.ftc.teamcode.CommandCode.Commands.*;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.*;


@TeleOp
public class Main extends CommandOpMode {
    ArmSubsystem arm;
    DroneSubsystem drone;
    HookSubsystem hook;
    DrivetrainSubsystem chassis;
    ImuSubsystem imu;

    GamepadEx pad1;
    GamepadEx pad2;

    public void initialize() {
        arm = new ArmSubsystem(hardwareMap);
        drone = new DroneSubsystem(hardwareMap);
        hook = new HookSubsystem(hardwareMap);
        chassis = new DrivetrainSubsystem(hardwareMap);
        imu = new ImuSubsystem(hardwareMap);


        pad1 = new GamepadEx(gamepad1);
        pad2 = new GamepadEx(gamepad2);

        // --- Defaults ---
        register(arm);
        arm.setDefaultCommand(new HoldArm(arm, CONFIG.CONTROL_SURFACES.ARM.TICKS));  

        // --- Gamepad1 ---
//        Does drivy stuff
        new StickTrigger(pad1, Stick.LEFT_X, CONFIG.CONTROLLER.STICK_DEADZONE)
                .or(new StickTrigger(pad1, Stick.RIGHT_Y, CONFIG.CONTROLLER.STICK_DEADZONE))
//                .or(pad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))
                .whileActiveContinuous(new MoveRobot(pad1, chassis, imu))
                .whenInactive(new InstantCommand(() -> {
                    chassis.stop();
                }, chassis));

        // --- Gamepad2 ---
        // Arm
        new StickTrigger(pad2, Stick.LEFT_Y, CONFIG.CONTROLLER.STICK_DEADZONE)
        .whileActiveContinuous(new MoveArm(pad2, arm));

        // Drone
        new GamepadButton(pad2, GamepadKeys.Button.X).whenHeld(new InstantCommand(() -> {
            drone.fire();
        }, drone)).whenReleased(new InstantCommand(() -> {
            drone.stop();
        }, drone));
        // Alternative
        // pad1.getGamepadButton(GamepadKeys.Button.X)

        // Hook
        new GamepadButton(pad2, GamepadKeys.Button.Y).whenHeld(new InstantCommand(() -> {
            hook.raise();
        }, hook)).whenReleased(new InstantCommand(() -> {
            hook.stop();
        }, hook));

        new GamepadButton(pad2, GamepadKeys.Button.A).whenHeld(new InstantCommand(() -> {
            hook.lower();
        }, hook)).whenReleased(new InstantCommand(() -> {
            hook.stop();
        }, hook));
    }
}