package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROL_SURFACES.CLAW;
public class ClawSubsystem extends SubsystemBase {
    private final CRServo svClaw1;
    private final CRServo svClaw2;

    public ClawSubsystem(HardwareMap hardMap) {
        svClaw1 = hardMap.get(CRServo.class, CLAW.DEVICE1);
        svClaw2 = hardMap.get(CRServo.class, CLAW.DEVICE2);

        svClaw1.setDirection(CLAW.DIR1);
        svClaw2.setDirection(CLAW.DIR2);
    }

    public void grip() {
        svClaw1.setPower(1);
        svClaw2.setPower(1);
    }

    public void release() {
        svClaw1.setPower(-1);
        svClaw2.setPower(-1);
    }

    public void stop() {
        svClaw1.setPower(0);
        svClaw2.setPower(0);
    }
}
