package org.firstinspires.ftc.teamcode.CommandCode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button;
import com.arcrobotics.ftclib.command.gamepad;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.CommandOpMode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.CONFIG;
import org.firstinspires.ftc.teamcode.CommandCode.Commands.*;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.*;


@TeleOp
public class Main extends CommandOpMode {
    final ArmSubsystem arm;
    final GamepadEx pad1;
    final GamepadEx pad2;

    public void initialize() {
        arm = new ArmSubsystem(hardwareMap, CONFIG.CONTROL_SURFACES.ARM.ARM1_DEVICE,CONFIG.CONTROL_SURFACES.ARM.ARM2_DEVICE);
        pad1 = new GamepadEx(gamepad1);
        pad2 = new GamepadEx(gamepad2);

        // Defaults
        register(arm);
        arm.setDefaultCommand(new HoldArm(arm, CONFIG.CONTROL_SURFACES.ARM.TICKS));  

        // Gamepad1

        // Gamepad2
        new StickTrigger(pad2, Stick.LEFT_Y, CONFIG.CONTROLLER.STICK_DEADZONE).whileActiveContinuous(new MoveArm(pad2, arm));
    }
}