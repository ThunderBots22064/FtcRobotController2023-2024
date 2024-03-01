/* Copyright (c) 2019 FIRST. All rights reserved.
 *
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

package org.firstinspires.ftc.teamcode.Auto;

import android.os.Build;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CONFIG;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "BlueAuto", group = "Concept")
//@Disabled
public class BlueAuto extends LinearOpMode {
    /*** CONFIG PRESETS ***/
    /* from old config, commented out bc not in use currently
    float SPEED_DEF = CONFIG.DRIVETRAIN.SPEED_DEF;
    float SPEED_MIN = CONFIG.DRIVETRAIN.SPEED_MIN;
    float SPEED_MAX = CONFIG.DRIVETRAIN.SPEED_MAX;
    float SPEED_VAR = CONFIG.DRIVETRAIN.SPEED_VAR;
    float SPEED_HOL = SPEED_DEF;
    float ARM_SPEED = CONFIG.CONTROL_SURFACES.ARM.ARM_SPEED;*/

    // Drivetrain motors
    DcMotor mtFL = null; // Front Left
    DcMotor mtFR = null; // Front Right
    DcMotor mtBL = null; // Back Left
    DcMotor mtBR = null; // Back Right

    // Arm Motor and Claw Servos
   /* from old config, commented out bc not in use currently
    CRServo svClaw1 = null;
    CRServo svClaw2 = null;
    DcMotor mtArm = null;
    int lastArmPos = 0;
    int expecArmPos = 0;
    float armProp = 0;
    float armDeriv = 0;
    float armInteg = 0;*/

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "blue.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private static final String TFOD_MODEL_FILE = Environment.getExternalStorageDirectory().getPath() + "/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "blue",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    //Stop function to stop power to wheels for inputted time (in seconds)
    private void stop(int time__in_seconds_) {
        mtBL.setPower(0);
        mtBR.setPower(0);
        mtFL.setPower(0);
        mtFR.setPower(0);
        sleep(time__in_seconds_ * 1000);
    }
    // Forward function to move wheels forward for inputted time and with inputted power
    private void forward(double time__in_seconds_, double power) {
        mtBL.setPower(power);
        mtBR.setPower(power);
        mtFL.setPower(power);
        mtFR.setPower(power);
        sleep((long) (time__in_seconds_ * 1000));
    }
    // Backward function to move wheels backward for inputted time and with inputted power
    private void backward(double time__in_seconds_, double power){
        mtBL.setPower(-power);
        mtBR.setPower(-power);
        mtFR.setPower(-power);
        mtFL.setPower(-power);
        sleep((long) (time__in_seconds_ * 1000));
    }
    // Turn left function to turn left for inputted time and with inputted power
    private void turn_Left(double time__in_seconds_, double power) {
        mtBL.setPower(-power);
        mtBR.setPower(power);
        mtFR.setPower(power);
        mtFL.setPower(-power);
        sleep((long) (time__in_seconds_ * 1000));
    }
    // Turn right function to turn right for inputted time and with inputted power
    private void turn_Right(double time__in_seconds_, double power){
        mtBL.setPower(power);
        mtBR.setPower(-power);
        mtFR.setPower(-power);
        mtFL.setPower(power);
        sleep((long) (time__in_seconds_ * 1000));
    }

    boolean have_seen = false;

    @Override
    public void runOpMode() {
        // Setup motors
        // Motors are first identified with the 'mt' (Motor)
        // They're then identified with F (Front) or B (Back)
        // Next the side is identified with L (Left) or R (Right)
        mtFL = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.FRONT_LEFT_DEVICE);
        mtFR = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.FRONT_RIGHT_DEVICE);
        mtBL = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.BACK_LEFT_DEVICE);
        mtBR = hardwareMap.get(DcMotor.class, CONFIG.DRIVETRAIN.BACK_RIGHT_DEVICE);

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

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // this is where run blocks would go if this was a block code
            forward(1, 0.25);
            turn_Right(0.5, 0.25);
            stop(2);

            while (opModeIsActive()) {
                // this is where loop blocks would go if this was a block code
                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Get a list of recognitions from TFOD.
                List<Recognition> currentRecognitions = tfod.getRecognitions();

                if (JavaUtil.listLength(currentRecognitions) == 0 && !have_seen) {
                    telemetry.addData("TFOD", "I'm blind");
                    turn_Left(0.5, 0.25);
                    stop(1);
                    telemetryTfod();
                    // Push telemetry to the Driver Station
                    telemetry.update();
                } else if (JavaUtil.listLength(currentRecognitions) > 0) {
                    telemetryTfod();
                    // Push telemetry to the Driver Station.
                    telemetry.update();
                    telemetry.addData("TFOD", "I'm feeling blue");
                    have_seen = true;
                    forward(1.6, 0.25);
                    backward(0.25, 0.25);
                    stop(10);
                }
            }
            }
        }
    // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            builder.setCameraResolution(new Size(640, 480));
        }

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.95f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class
