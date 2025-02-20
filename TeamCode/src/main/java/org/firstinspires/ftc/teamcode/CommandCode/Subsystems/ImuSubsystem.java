package org.firstinspires.ftc.teamcode.CommandCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.CONFIG.CONTROL_SURFACES.INTERTIALMU;


public class ImuSubsystem extends SubsystemBase {
//    private final RevIMU imu;
    private final IMU imu;


    /**
     * Creates an IMU subsystem for managing the onboard Inertial Measurement Unit
     * @param hardMap the hardware map object
     */
    public ImuSubsystem(HardwareMap hardMap) {
//        imu = new RevIMU(hardMap, INTERTIALMU.IMU_DEVICE);
//        imu.init();
        imu = hardMap.get(IMU.class, INTERTIALMU.DEVICE);

        imu.initialize(
            new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            ))
        );
    }

    /**
     * Gets the IMU's heading scaled for 0 - 360 degrees
     * @return the IMU's scaled heading in degrees
     */
    public double getHeading() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); /*imu.getRotation2d().getDegrees(); */

        // Converts the -180 to 180 degrees into a 0 to 360 degrees value
        if (heading < 0) {
            heading += 360;
        }

        return heading;
    }

    /**
     * Gets the raw yaw value read off the IMU
     * @return the IMU yaw value in degrees
     */
    public double getHeadingRaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); /*imu.getRotation2d().getDegrees(); */
    }

    /**
     * Resets the IMU's heading to the current heading
     */
    public void resetHeading() {
        imu.resetYaw();
    }
}
