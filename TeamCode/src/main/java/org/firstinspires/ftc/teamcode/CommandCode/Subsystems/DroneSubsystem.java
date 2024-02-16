package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROL_SURFACES.DRONE;
import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class DroneSubsystem extends SubsystemBase {
    private final CRServo svDrone;

    public DroneSubsystem(final HardwareMap hardMap) {
        svDrone = hardMap.get(CRServo.class, DRONE.DEVICE);

        svDrone.setDirection(DRONE.DIR);
    }

    public void fire() {
        svDrone.setPower(1);
    }

    public void stop() {
        svDrone.setPower(0);
    }
}