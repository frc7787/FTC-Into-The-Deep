package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.vision.SampleColorDetector.SampleColor.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SampleColorDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Test - Sample Color Detector", group = "Test")
public class SampleColorDetectorTest extends CommandOpMode {
    public OpenCvCamera camera;


    @Override public void initialize() {
        SampleColorDetector colorDetector = new SampleColorDetector(YELLOW);

        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"),
                        cameraMonitorViewId
                );

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(colorDetector);
            }

            @Override public void onError(int errorCode) {
                telemetry.addData("Failed to open camera due to error code", errorCode);
                telemetry.update();
            }
        });
    }
}
