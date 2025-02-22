package org.firstinspires.ftc.teamcode.CommandCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ArmSubsystem;

/**
 * This command move the arm to a certain position given a joystick input
*/
public class MoveArm extends CommandBase {
    private final ArmSubsystem arm;
    private final GamepadEx pad;

    /**
     * Creates a MoveArm command
     * @param arm The arm subsystem
     */
    public MoveArm(final GamepadEx pad, final ArmSubsystem arm) {
        this.arm = arm;
        this.pad = pad;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        double stick = pad.getLeftY();

        if (stick > 0) {
            arm.raise();
        } else if (stick < 0) {
            arm.lower();
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}