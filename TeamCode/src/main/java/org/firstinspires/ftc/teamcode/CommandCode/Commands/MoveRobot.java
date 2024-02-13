package org.firstinspires.ftc.teamcode.CommandCode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ImuSubsystem;

public class MoveRobot extends CommandBase {
    private final DrivetrainSubsystem chassis;
    private final ImuSubsystem imu;
    private final GamepadEx pad;
    public MoveRobot(GamepadEx pad, DrivetrainSubsystem chassis, ImuSubsystem imu) {
        this.pad = pad;
        this.chassis = chassis;
        this.imu = imu;

        addRequirements(chassis);
        addRequirements();
    }

    @Override
    public void execute() {
        double turn = (pad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < pad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) ? 1.0 : -1.0;
        chassis.driveFieldCentric(pad.getLeftX(), pad.getRightY(), turn, imu.getHeading());
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
}
