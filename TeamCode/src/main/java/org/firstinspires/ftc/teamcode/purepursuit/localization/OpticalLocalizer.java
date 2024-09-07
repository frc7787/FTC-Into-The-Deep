package org.firstinspires.ftc.teamcode.purepursuit.localization;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static  org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.*;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

public final class OpticalLocalizer implements Localizer {
    private final KalmanFilter filter;

    private final SparkFunOTOS opticalOdometrySensor;

    /**
     * Pose before kalman filter is applied, should only be used for debugging the filter
     */
    public Pose2D rawPose;

    /**
     * The pose value after the kalman filter is applied
     */
    public Pose2D pose;

    /**
     * The current velocity of the imu
     */
    public Pose2D velocity;

    public OpticalLocalizer(@NonNull HardwareMap hardwareMap) {
        opticalOdometrySensor = hardwareMap.get(SparkFunOTOS.class, OPTICAL_ODOMETRY_NAME);

        filter = new KalmanFilter(Q, R, N);

        configureIMU();

        rawPose  = new Pose2D(0,0,0);
        pose     = new Pose2D(0,0,0);
        velocity = new Pose2D(0,0,0);
    }

    private void configureIMU() {
        opticalOdometrySensor.resetTracking();
        opticalOdometrySensor.calibrateImu(IMU_CALIBRATION_SAMPLES, false);
        opticalOdometrySensor.setAngularScalar(ANGULAR_SCALAR);
        opticalOdometrySensor.setLinearScalar(LINEAR_SCALAR);

        pose     = new Pose2D(0,0,0);
        velocity = new Pose2D(0,0,0);
    }

    public OpticalLocalizer(SparkFunOTOS opticalOdometrySensor, Pose2D pose) {
        this.opticalOdometrySensor = opticalOdometrySensor;
        this.pose = pose;
    }

    @Override public void update() {
        Pose2D rawPose = opticalOdometrySensor.getPosition();
        pose = new Pose2D(
                filter.estimate(rawPose.x),
                filter.estimate(rawPose.y),
                filter.estimate(rawPose.h)
        );
        velocity = opticalOdometrySensor.getVelocity();
    }

    @Override public void reset() {
        opticalOdometrySensor.resetTracking();
    }

    @Override public void setPosition(SparkFunOTOS.Pose2D pose) {
        this.pose    = pose;
        this.rawPose = pose;
        opticalOdometrySensor.setPosition(pose);
    }

    @Override public void debug(Telemetry telemetry) {
        telemetry.addData("Raw X", rawPose.x);
        telemetry.addData("Raw Y", rawPose.y);
        telemetry.addData("Raw H", rawPose.h);
        telemetry.addData("X", pose.x);
        telemetry.addData("Y", pose.y);
        telemetry.addData("H", pose.h);
        telemetry.addData("X Velocity", velocity.x);
        telemetry.addData("Y Velocity", velocity.y);
        telemetry.addData("H Velocity", velocity.h);
    }
}
