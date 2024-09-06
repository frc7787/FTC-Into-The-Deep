package org.firstinspires.ftc.teamcode.purepursuit.localization;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Localizer {
    void update();

    void reset();

    void setPosition(@NonNull Pose2D pose2D);

    void debug(@NonNull Telemetry telemetry);
}
