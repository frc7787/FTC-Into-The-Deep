package org.firstinspires.ftc.teamcode.vision.apriltags;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.*;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

import java.util.*;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Utility - April Tag Tuning", group = "Utility")
public class AprilTagTuning extends OpMode {
    private AprilTagProcessor aprilTagProcessor;
    private DataLogger aprilTagLogger;

    private static final int DESIRED_TAG_ID = 5; // -1 locks on to any tag

    private VisionPortal visionPortal;

    private int minExposure, maxExposure, exposure;

    private int minGain, gain, maxGain;

    private int minWhiteBalance, whiteBalance, maxWhiteBalance;

    private Gamepad currentGamepad, prevGamepad;

    @Override public void init() {
        initAprilTagDetection();
        getDefaultCameraSettings();

        aprilTagLogger = new DataLogger("AprilTagLogger");

        exposure = 2;
        gain = 0;
        whiteBalance = 4000;

        setCameraProperties(exposure, gain, whiteBalance);

        prevGamepad    = new Gamepad();
        currentGamepad = new Gamepad();
    }

    @Override public void loop() {
        prevGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        telemetry.addData("Exposure", exposure);
        telemetry.addData("Gain", gain);
        telemetry.addData("White Balance", whiteBalance);

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        int numberOfTags = currentDetections.size();

        if (numberOfTags > 0 ) {
            telemetry.addData("Number of Tags Detected", currentDetections.size());
        } else {
            telemetry.addLine("No tags currently detected");
        }

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null) continue;

            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                allAprilTagDetections.add(detection);

                telemetry.addData("Detected Tag", detection.id);
                telemetry.addData("Distance From Tag", detection.ftcPose.range);
                telemetry.addData("Yaw", detection.ftcPose.yaw);
                telemetry.addData("Pitch", detection.ftcPose.pitch);
                telemetry.addData("Roll", detection.ftcPose.roll);
            }
        }

        listenForCameraPropertyAdjustments();

        exposure = Range.clip(exposure, minExposure, maxExposure);
        gain = Range.clip(gain, minGain, maxGain);
        whiteBalance = Range.clip(whiteBalance, minWhiteBalance, maxWhiteBalance);

        setCameraProperties(exposure, gain, whiteBalance);

        telemetry.update();

        if(gamepad1.options){
            saveAprilTagData();
        }
    }

    @Override public void stop() {

    }

    private void initAprilTagDetection() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setLensIntrinsics(660.750, 660.75, 323.034, 230.681) // C615 measured kk Dec 5 2023
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .setAutoStopLiveView(true)
                .build();
    }

    private void getDefaultCameraSettings() {
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) { // Wait for camera to start streaming
            telemetry.addLine("Waiting for camera");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        minWhiteBalance = whiteBalanceControl.getMinWhiteBalanceTemperature();
        maxWhiteBalance = whiteBalanceControl.getMaxWhiteBalanceTemperature();
        whiteBalance = whiteBalanceControl.getWhiteBalanceTemperature();
    }

    private void listenForCameraPropertyAdjustments() {
        if (currentGamepad.left_bumper && !prevGamepad.left_bumper) exposure++;

        if (currentGamepad.left_trigger > 0.5 && !(prevGamepad.left_trigger > 0.5)) exposure--;

        if (currentGamepad.right_bumper && !prevGamepad.right_bumper) gain++;

        if (currentGamepad.right_trigger > 0.5 && !(prevGamepad.right_trigger > 0.5)) gain--;

        if (currentGamepad.dpad_up && !prevGamepad.dpad_up) whiteBalance++;

        if (currentGamepad.dpad_down && !prevGamepad.dpad_down) whiteBalance--;

        if (currentGamepad.dpad_left && !prevGamepad.dpad_left) whiteBalance += 1;

        if (currentGamepad.dpad_right && !prevGamepad.dpad_right) whiteBalance -= 10;
    }

    private void setCameraProperties(int exposureMS, int gain, int white) {
        // Wait for camera to start streaming
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for camera");
            telemetry.update();
        }

        // Make sure that the telemetry clears properly after we exit the while loop
        telemetry.update();

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        whiteBalanceControl.setWhiteBalanceTemperature(white);
    }
}