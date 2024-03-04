package org.firstinspires.ftc.teamcode.CommandCode.Auto;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import org.firstinspires.ftc.teamcode.CONFIG;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.CommandCode.Subsystems.DrivetrainSubsystem;

@Autonomous(name="Red Backboard", group="Linear OpMode")
public class redBackboard extends LinearOpMode {
    ClawSubsystem claw;
    ArmSubsystem arm;
    DrivetrainSubsystem drivetrain;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private void stop(int time__in_seconds_) {
        drivetrain.stop();
        sleep((time__in_seconds_ * 1000));
    }

    private void turn_Right(double time__in_seconds_, double power) {
        drivetrain.driveRobotCentricRaw(0, 0, power, false);
        sleep((long) (time__in_seconds_ * 1000));
    }

    private void backward(double time__in_seconds_, double power) {
        drivetrain.driveRobotCentricRaw(0, -power, 0, false);
        sleep((long) (time__in_seconds_ * 1000));
    }

    private void forward(double time__in_seconds_, double power) {
        drivetrain.driveRobotCentricRaw(0, power, 0, false);
        sleep((long) (time__in_seconds_ * 1000));
    }

    @Override
    public void runOpMode() {
        claw = new ClawSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //go forward 150ms-stop-turn right 300ms-go forward 3000ms- go back 150 ms?
            arm.raise();
            sleep(250);
            arm.stop();
            forward(0.5, 0.25);
            stop(1);
            turn_Right(2, 0.25);
            forward(5, 0.25);
            claw.release();
            sleep(250);
            claw.stop();
            sleep(10);
            backward(0.15, 0.25);
            stop(20);
            break;
        }
    }
}