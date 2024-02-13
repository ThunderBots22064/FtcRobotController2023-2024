package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROL_SURFACES.INTERTIALMU;


public class ImuSubsystem extends SubsystemBase {
    private final IMU imu;


    public ImuSubsystem(HardwareMap hardMap) {
        imu = hardMap.get(IMU.class, INTERTIALMU.IMU_DEVICE);

        imu.initialize(
            new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            ))
        );
    }

    public double getHeading() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        /*
        // Converts the -180 to 180 degrees into a 0 to 360 degrees value
        if (heading < 0) {
            heading -= heading;
            heading += 180;
        }
         */

        return heading;
    }
}
