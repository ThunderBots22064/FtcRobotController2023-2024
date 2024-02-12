package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROL_SURFACES.ARM;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotor mtArm1;
    private final DcMotor mtArm2;

    public ArmSubsystem(final HardwareMap hardMap, final String motor1, final String motor2) {
        mtArm1 = hardMap.get(DcMotor.class, motor1);
        mtArm2 = hardMap.get(DcMotor.class, motor2);

        mtArm1.setDirection(ARM.ARM1_DIR);
        mtArm1.setDirection(ARM.ARM2_DIR);

        mtArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void raise() {
        mtArm1.setPower(ARM.ARM_SPEED);
        mtArm2.setPower(ARM.ARM_SPEED);
    }

    public void lower() {
        mtArm1.setPower(-ARM.ARM_SPEED);
        mtArm2.setPower(-ARM.ARM_SPEED);
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