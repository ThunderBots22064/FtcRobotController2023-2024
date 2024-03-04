package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROL_SURFACES.ARM;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotor mtArm1;
    private final DcMotor mtArm2;

    public ArmSubsystem(final HardwareMap hardMap) {
        mtArm1 = hardMap.get(DcMotor.class, ARM.DEVICE1);
        mtArm2 = hardMap.get(DcMotor.class, ARM.DEVICE2);

        mtArm1.setDirection(ARM.DIR1);
        mtArm1.setDirection(ARM.DIR2);

        mtArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void raise() {
        mtArm1.setPower(ARM.SPEED);
        mtArm2.setPower(ARM.SPEED);
    }

    public void lower() {
        mtArm1.setPower(-ARM.SPEED);
        mtArm2.setPower(-ARM.SPEED);
    }

    public int getAvgPos() {
        return (mtArm1.getCurrentPosition() + mtArm2.getCurrentPosition()) / 2;
    }

    /**
     * Attempts to keep move the arm to a target position using a P algorithim
     * @param targetPos The encoder position to try to reach
     */
    public void target(final int targetPos) {
        int avgPos = getAvgPos();
        int error = targetPos - avgPos;
        float total = 0;

        float proportion = ARM.Kp * error;
        
        // Thank you!
        // https://stackoverflow.com/questions/16656651/does-java-have-a-clamp-function
        if (proportion > ARM.CLAMP) {
            proportion = ARM.CLAMP;
        } else if (proportion < -ARM.CLAMP) {
            proportion = -ARM.CLAMP;
        }

        total += proportion;

        mtArm1.setPower(total);
        mtArm2.setPower(total);
    }

    public void stop() {
        mtArm1.setPower(0);
        mtArm2.setPower(0);
    }
}