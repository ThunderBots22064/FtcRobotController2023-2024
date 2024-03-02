package org.firstinspires.ftc.teamcode.CommandCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ImuSubsystem;

import java.lang.Math;
import java.util.function.DoubleSupplier;

public class MoveRobot extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    private final double strafeTarget;
    private final double vertTarget;
    private final double rotTarget;

    public MoveRobot(final DrivetrainSubsystem drivetrain, final double strafeTarget, final double vertTarget, final double rotTarget) {
        this.drivetrain = drivetrain;

        this.strafeTarget = strafeTarget;
        this.vertTarget = vertTarget;
        this.rotTarget = rotTarget;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }
}
