package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Test - Spline", group = "Test")
public final class SplineTest extends LinearOpMode {
    public static Pose2d startPosition = new Pose2d(0,0,0);

    @Override public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPosition);

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(startPosition)
                    .splineTo(new Vector2d(30, 30), Math.PI / 2)
                    .splineTo(new Vector2d(0, 60), Math.PI)
                    .build());
    }
}
