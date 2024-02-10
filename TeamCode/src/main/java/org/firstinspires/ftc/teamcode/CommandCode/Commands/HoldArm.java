package org.firstinspires.ftc.teamcode.CommandCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ArmSubsystem;

/**
 * This command holds the robot's arm at certain position
*/
public class HoldArm extends CommandBase {
    private final ArmSubsystem arm;
    private final int targetPos;
    private final int ticks;
    private int tickCount = 0;

    /**
     * Creates a HoldArm command
     * @param arm The arm subsystem
     * @param ticks The number of ticks between each correction
     */
    public HoldArm(final ArmSubsystem arm, final int ticks) {
        this.arm = arm;
        this.ticks = ticks;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        targetPos = arm.getAvgPos();
    }

    @Override
    public void execute() {
        if (tickCount >= ticks) {
            arm.target(targetPos);
        }
    }

    @Override
    public void end() {
        arm.stop();
    }
}