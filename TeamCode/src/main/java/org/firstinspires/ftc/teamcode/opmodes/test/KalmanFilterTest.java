package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.purepursuit.localization.OpticalLocalizer;
import static org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.*;

@TeleOp(name = "Test - Kalman Filter", group = "Test")
public final class KalmanFilterTest extends OpMode {
    private OpticalLocalizer localizer;

    @Override public void init() {
       SparkFunOTOS opticalOdometry = hardwareMap.get(SparkFunOTOS.class, OPTICAL_ODOMETRY_NAME);

       localizer = new OpticalLocalizer(opticalOdometry);
    }

    @Override public void loop() {
        localizer.debug(telemetry);
    }
}
