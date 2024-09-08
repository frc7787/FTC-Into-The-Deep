package org.firstinspires.ftc.teamcode.vision.apriltags;

import android.util.Size;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.*;
import org.firstinspires.ftc.teamcode.utility.DataLogger;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.AprilTagConstants.*;

import java.util.*;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Utility - April Tag Tuning", group = "Utility")
public class AprilTagTuning extends CommandOpMode {
    private AprilTagProcessor aprilTagProcessor;
    private DataLogger aprilTagLogger;

    private static final int DESIRED_TAG_ID = 5; // -1 locks on to any tag

    private VisionPortal visionPortal;

    private int minExposure, maxExposure, exposure;
    private int minGain, gain, maxGain;
    private int minWhiteBalance, whiteBalance, maxWhiteBalance;

    @Override public void initialize() {
        initializeAprilTagDetection();
        initializeCamera();

        aprilTagLogger = new DataLogger("AprilTagLogFile");

        schedule(
                new RunCommand(this::updateDetections),
                new RunCommand(telemetry::update)
        );
    }

    private void configureBindings() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
    }

    private void updateDetections() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        int numberOfTags = currentDetections.size();

        if (numberOfTags > 0 ) {
            telemetry.addData("Number of Tags Detected", currentDetections.size());
        } else {
            telemetry.addLine("No tags currently detected");
        }

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null) continue;

            if (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID) {
                aprilTagLogger.addLine(detectionAsString(detection));

                telemetry.addData("Detected Tag", detection.id);
                telemetry.addData("Distance From Tag", detection.ftcPose.range);
                telemetry.addData("Yaw", detection.ftcPose.yaw);
                telemetry.addData("Pitch", detection.ftcPose.pitch);
                telemetry.addData("Roll", detection.ftcPose.roll);
            }
        }
    }


    private String detectionAsString(AprilTagDetection detection) {
       return System.currentTimeMillis() + "," + detection.ftcPose.yaw + ","
              + detection.ftcPose.pitch + "," + detection.ftcPose.roll + ","
              + detection.ftcPose.range + "," + detection.ftcPose.bearing;
    }

    private void initializeAprilTagDetection() {
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

    private void initializeCamera() {
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for camera");
            telemetry.update();
        }

        // Ensure telemetry is updated properly after the loop
        telemetry.update();

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
        exposure    = Range.clip(EXPOSURE_MS, minExposure, maxExposure);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();
        gain    = Range.clip(GAIN, minGain, maxGain);

        WhiteBalanceControl whiteBalanceControl
                = visionPortal.getCameraControl(WhiteBalanceControl.class);

        minWhiteBalance = whiteBalanceControl.getMinWhiteBalanceTemperature();
        maxWhiteBalance = whiteBalanceControl.getMaxWhiteBalanceTemperature();
        whiteBalance    = Range.clip(WHITE_BALANCE, minWhiteBalance, maxWhiteBalance);
    }

    private void setCameraProperties(int exposureMS, int gain, int whiteBalance) {
        // Wait for camera to start streaming
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for camera");
            telemetry.update();
        }

        // Ensure telemetry clears properly after we exit the while loop
        telemetry.update();

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(
                Range.clip(exposureMS, minExposure, maxExposure),
                TimeUnit.MILLISECONDS
        );

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(Range.clip(gain, minGain, maxGain));

        WhiteBalanceControl whiteBalanceControl
                = visionPortal.getCameraControl(WhiteBalanceControl.class);
        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        whiteBalanceControl.setWhiteBalanceTemperature(
                Range.clip(whiteBalance, minWhiteBalance, maxWhiteBalance));
    }
}