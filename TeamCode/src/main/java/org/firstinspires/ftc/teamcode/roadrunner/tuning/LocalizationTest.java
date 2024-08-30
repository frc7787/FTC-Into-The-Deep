package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.*;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.roadrunner.*;

@TeleOp(name = "Test - Localization", group = "Test")
public final class LocalizationTest extends LinearOpMode {
    private MecanumDrive drive;
    private MultipleTelemetry multipleTelemetry;

    @Override public void runOpMode() {
        multipleTelemetry
                = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            updateTelemetry();
        }
    }

    private void updateTelemetry() {
        multipleTelemetry.addData("X Position", drive.pose.position.x);
        multipleTelemetry.addData("Y Position", drive.pose.position.y);
        multipleTelemetry.addData(
                "Heading (Degrees)", Math.toDegrees(drive.pose.heading.toDouble()));
        multipleTelemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
