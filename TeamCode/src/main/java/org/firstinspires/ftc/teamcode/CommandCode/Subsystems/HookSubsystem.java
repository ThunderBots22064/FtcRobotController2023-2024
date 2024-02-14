package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROL_SURFACES.HOOK;
public class HookSubsystem extends SubsystemBase {
    private final DcMotor mtHook;
    
    public HookSubsystem(HardwareMap hardMap) {
        mtHook = hardMap.get(DcMotor.class, HOOK.HOOK_DEVICE);

        mtHook.setDirection(HOOK.HOOK_DIR);

        mtHook.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void raise() {
        mtHook.setPower(HOOK.HOOK_SPEED_UP);
    }

    public void lower() {
        mtHook.setPower(-HOOK.HOOK_SPEED_DOWN);
    }

    public void stop() {
        mtHook.setPower(0);
    }
}
