package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROL_SURFACES.DRONE;
import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class DroneSubsystem extends SubsystemBase {
    private final CRServo servo;

    public DroneSubsystem(final HardwareMap hardMap, final String servo) {
        this.servo = hardMap.get(CRServo.class, servo);

        this.servo.setDirection(DRONE.DRONE_DIR);
    }

    public void fire() {
        servo.setPower(1);
    }

    public void stop() {
        servo.setPower(0);
    }
}