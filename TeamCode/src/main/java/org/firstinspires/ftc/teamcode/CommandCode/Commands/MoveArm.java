package org.firstinspires.ftc.teamcode.CommandCode.Commands;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROL_SURFACES.ARM;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ArmSubsystem;
import com.arcrobotics.ftclib.command.gamepad;

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
    public MoveArm(final ArmSubsystem arm, final GamepadEx pad) {
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
    public void end() {
        arm.stop();
    }
}