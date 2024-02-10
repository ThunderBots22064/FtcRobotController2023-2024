package org.firstinspires.ftc.teamcode.CommandCode;

import com.arcrobotics.ftclib.command.button;
import com.arcrobotics.ftclib.command.gamepad;

enum Stick {
    LEFT_X,
    LEFT_Y,
    RIGHT_X,
    RIGHT_Y
};

public class StickTrigger extends Trigger {
    final axis controlAxis;
    final double deadzone;

    /**
     * Creates a StickTrigger trigger, if a joystick's axis moves out of the deadzone (+/-) it will become true
     * @param pad the gamepad to use
     * @param stick the gamepad stick and axis
     * @param deadzone the minimum value where the stick will be considered active
     */
    public StickTrigger(final GamepadEx pad, final Stick stick, final double deadzone) {
        this.deadzone = deadzone;
        
        switch (stick) {
            case LEFT_X:
                controlAxis = pad::getLeftX;
                break;
            case LEFT_Y:
                controlAxis = pad::getLeftY;
                break;
            case RIGHT_X:
                controlAxis = pad::getRightX;
                break;
            case RIGHT_Y:
                controlAxis = pad::getRightY;
                break;
            default:
                controlAxis = () -> {return 0;};
                break;
        }
    }

    @Override
    public boolean get() {
        double pos = controlAxis.getPos();

        if (pos >= deadzone || pos <= -deadzone) {
            return true;
        }
        return false;
    }

    @FunctionalInterface
    private interface axis {
        double getPos();
    }
}