package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROL_SURFACES.ARM;

import com.arcrobotics.ftclib2.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotor mtArm1;
    private final DcMotor mtArm2;

    public ArmSubsystem(final HardwareMap hardMap, final String motor1, final String motor2) {
        mtArm1 = hardMap.get(DcMotor.class, motor1);
        mtArm2 = hardMap.get(DcMotor.class, motor2);

        mtArm1.setDirection(ARM.ARM1_DIR);
        mtArm1.setDirection(ARM.ARM2_DIR);
    }

    public void raise() {
        mtArm1.setPower(ARM.ARM_SPEED);
        mtArm2.setPower(ARM.ARM_SPEED);
    }

    public void lower() {
        mtArm1.setPower(-ARM.ARM_SPEED);
        mtArm2.setPower(-ARM.ARM_SPEED);
    }

    public void stop() {
        mtArm1.setPower(0);
        mtArm2.setPower(0);
    }
}