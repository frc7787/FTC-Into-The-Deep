package org.firstinspires.ftc.teamcode.purepursuit.localization;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static  org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.*;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

public final class OpticalLocalizer implements Localizer {
    private final SparkFunOTOS opticalOdometrySensor;

    @Nullable
    private final Telemetry telemetry;

    public Pose2D pose, velocity;

    public OpticalLocalizer(@NonNull HardwareMap hardwareMap) {
        opticalOdometrySensor = hardwareMap.get(SparkFunOTOS.class, OPTICAL_ODOMETRY_NAME);

        initialize();

        pose     = new Pose2D(0,0,0);
        velocity = new Pose2D(0,0,0);

        telemetry = null;
    }

    public OpticalLocalizer(HardwareMap hardwareMap, Pose2D pose) {
        opticalOdometrySensor = hardwareMap.get(SparkFunOTOS.class, OPTICAL_ODOMETRY_NAME);

        initialize();

        this.pose = pose;
        velocity  = new Pose2D(0,0,0);

        telemetry = null;
    }

    public OpticalLocalizer(HardwareMap hardwareMap, Pose2D pose, @Nullable Telemetry telemetry) {
        opticalOdometrySensor = hardwareMap.get(SparkFunOTOS.class, OPTICAL_ODOMETRY_NAME);

        initialize();

        this.pose = pose;
        velocity  = new Pose2D(0,0,0);

        this.telemetry = telemetry;
    }

    public OpticalLocalizer(HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        opticalOdometrySensor = hardwareMap.get(SparkFunOTOS.class, OPTICAL_ODOMETRY_NAME);

        initialize();

        this.pose = new Pose2D(0,0,0);
        velocity  = new Pose2D(0,0,0);

        this.telemetry = telemetry;
    }

    private void initialize() {
        opticalOdometrySensor.resetTracking();
        opticalOdometrySensor.calibrateImu(IMU_CALIBRATION_SAMPLES, false);

        opticalOdometrySensor.setLinearUnit(DistanceUnit.INCH);
        opticalOdometrySensor.setAngularUnit(AngleUnit.DEGREES);

        opticalOdometrySensor.setAngularScalar(ANGULAR_SCALAR);
        opticalOdometrySensor.setLinearScalar(LINEAR_SCALAR);

        opticalOdometrySensor.setOffset(OFFSET);
    }

    @Override public void update() {
        pose     = opticalOdometrySensor.getPosition();
        velocity = opticalOdometrySensor.getVelocity();
    }

    @Override public void reset() {
        opticalOdometrySensor.resetTracking();
    }

    @Override public void setPosition(SparkFunOTOS.Pose2D pose) {
        this.pose = pose;
        opticalOdometrySensor.setPosition(pose);
    }

    @Override public void debug() {
        if (telemetry == null) return;

        telemetry.addData("X", pose.x);
        telemetry.addData("Y", pose.y);
        telemetry.addData("H", pose.h);
        telemetry.addData("X Velocity", velocity.x);
        telemetry.addData("Y Velocity", velocity.y);
        telemetry.addData("H Velocity", velocity.h);
    }
}
