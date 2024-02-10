package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROL_SURFACES.DRONE;
import com.arcrobotics.ftclib2.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.hardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class DroneSubsystem extends SubsystemBase {
    private final CRServo servo;

    public DroneSubsystem(final HardwareMap hardMap, final String servo) {
        this.servo = hardMap.get(CRServo.class, servo);

        svDrone.setDirection(DRONE.DRONE_DIR);
    }

    public void fire() {
        svDrone.setPower(1);
    }

    public void stop() {
        svDrone.setPower(0);
    }
}