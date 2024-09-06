package org.firstinspires.ftc.teamcode.purepursuit.localization;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static  org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.*;

public final class OpticalLocalizer implements Localizer {
    private final SparkFunOTOS opticalOdometrySensor;

    public Pose2D pose, velocity;

    public OpticalLocalizer(@NonNull SparkFunOTOS opticalOdometrySensor) {
        this.opticalOdometrySensor = opticalOdometrySensor;

        opticalOdometrySensor.resetTracking();
        opticalOdometrySensor.calibrateImu(IMU_CALIBRATION_SAMPLES, false);
        opticalOdometrySensor.setAngularScalar(ANGULAR_SCALAR);
        opticalOdometrySensor.setLinearScalar(LINEAR_SCALAR);

        pose     = new Pose2D(0,0,0);
        velocity = new Pose2D(0,0,0);
    }

    public OpticalLocalizer(@NonNull SparkFunOTOS opticalOdometrySensor, @NonNull Pose2D pose) {
        this.opticalOdometrySensor = opticalOdometrySensor;
        this.pose = pose;
    }

    @Override public void update() {
        pose = opticalOdometrySensor.getPosition();
    }

    @Override public void reset() {
        opticalOdometrySensor.resetTracking();
    }

    @Override public void setPosition(@NonNull SparkFunOTOS.Pose2D pose) {
        this.pose = pose;
        opticalOdometrySensor.setPosition(pose);
    }

    @Override public void debug(@NonNull Telemetry telemetry) {
        telemetry.addData("X", pose.x);
        telemetry.addData("Y", pose.y);
        telemetry.addData("H", pose.h);
        telemetry.addData("X Velocity", velocity.x);
        telemetry.addData("Y Velocity", velocity.y);
        telemetry.addData("H Velocity", velocity.h);
    }
}
