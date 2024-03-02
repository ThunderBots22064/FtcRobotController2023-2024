package org.firstinspires.ftc.teamcode.CommandCode;/* Copyright (c) 2017 FIRST. All rights reserved.
 /*
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.RunCommand;

//import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.CONFIG;
//import org.firstinspires.ftc.teamcode.CommandCode.Commands.*;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.*;

//import org.firstinspires.ftc.teamcode.CONFIG;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Red Backboard", group="Linear OpMode")
public class redBackboard extends LinearOpMode {

    // Drivetrain motors
    DcMotor mtFL = null; // Front Left
    DcMotor mtFR = null; // Front Right
    DcMotor mtBL = null; // Back Left
    DcMotor mtBR = null; // Back Right

    ClawSubsystem claw;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private void stop(int time__in_seconds_) {
        mtBL.setPower(0);
        mtBR.setPower(0);
        mtFL.setPower(0);
        mtFR.setPower(0);
        sleep((time__in_seconds_ * 1000));
    }
    private void turn_Right(double time__in_seconds_, double power) {
        mtBL.setPower(power);
        mtBR.setPower(-power);
        mtFR.setPower(-power);
        mtFL.setPower(power);
        sleep((long) (time__in_seconds_ * 1000));
    }
    private void backward(double time__in_seconds_, double power){
        mtBL.setPower(-power);
        mtBR.setPower(-power);
        mtFR.setPower(-power);
        mtFL.setPower(-power);
        sleep((long) (time__in_seconds_ * 1000));
    }
    private void forward(double time__in_seconds_, double power) {
        mtBL.setPower(power);
        mtBR.setPower(power);
        mtFL.setPower(power);
        mtFR.setPower(power);
        sleep((long) (time__in_seconds_ * 1000));
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Setup motors
        // Motors are first identified with the 'mt' (Motor)
        // They're then identified with F (Front) or B (Back)
        // Next the side is identified with L (Left) or R (Right)
        mtFL = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.FRONT_LEFT_DEVICE);
        mtFR = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.FRONT_RIGHT_DEVICE);
        mtBL = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.BACK_LEFT_DEVICE);
        mtBR = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.BACK_RIGHT_DEVICE);

        claw = new ClawSubsystem(hardwareMap);

        mtFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mtFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtFL.setDirection(CONFIG.DRIVETRAIN.FL_DIR);
        mtFR.setDirection(CONFIG.DRIVETRAIN.FR_DIR);
        mtBL.setDirection(CONFIG.DRIVETRAIN.BL_DIR);
        mtBR.setDirection(CONFIG.DRIVETRAIN.BR_DIR);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //go forward 150ms-stop-turn right 300ms-go forward 3000ms- go back 150 ms?
            forward(0.15, 0.25);
            stop(1);
            turn_Right(0.3, 0.25);
            forward(3, 0.25);
            claw.stop();
            sleep(1);
            backward(0.15, 0.25);
            stop(20);

        }
    }
}