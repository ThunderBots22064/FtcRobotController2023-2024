package org.firstinspires.ftc.teamcode.CommandCode;

import com.arcrobotics.ftclib.command.button.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.CONFIG;
import org.firstinspires.ftc.teamcode.CommandCode.Commands.*;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.*;


@TeleOp
public class Main extends CommandOpMode {
    ArmSubsystem arm;
    DroneSubsystem drone;

    GamepadEx pad1;
    GamepadEx pad2;

    public void initialize() {
        arm = new ArmSubsystem(hardwareMap, CONFIG.CONTROL_SURFACES.ARM.ARM1_DEVICE,CONFIG.CONTROL_SURFACES.ARM.ARM2_DEVICE);
        drone = new DroneSubsystem(hardwareMap, CONFIG.CONTROL_SURFACES.DRONE.DRONE_DEVICE);
        pad1 = new GamepadEx(gamepad1);
        pad2 = new GamepadEx(gamepad2);

        // --- Defaults ---
        register(arm);
        arm.setDefaultCommand(new HoldArm(arm, CONFIG.CONTROL_SURFACES.ARM.TICKS));  

        // --- Gamepad1 ---

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
    }
}