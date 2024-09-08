package org.firstinspires.ftc.teamcode.constants;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public final class Constants {

    public static class FileConstants {
        @SuppressLint("sdCardPath")
        public static final String SD_CARD_PATH
                = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/";

        public static final String CONSTANTS_FILE_PATH = SD_CARD_PATH + "Constants/";
        public static final String APRIL_TAG_LOG_FILE_PATH = SD_CARD_PATH + "AprilTagLogs/";
    }

    public static class DrivebaseConstants {
        public static String FRONT_LEFT_DRIVE_MOTOR_NAME  = "frontLeftDriveMotor";
        public static String FRONT_RIGHT_DRIVE_MOTOR_NAME = "frontRightDriveMotor";
        public static String BACK_LEFT_DRIVE_MOTOR_NAME   = "backLeftDriveMotor";
        public static String BACK_RIGHT_DRIVE_MOTOR_NAME  = "backRightDriveMotor";

        public static String IMU_NAME = "imu";
        public static IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        public static double DRIVE_DEAD_ZONE  = 0.05;
        public static double STRAFE_DEAD_ZONE = 0.05;
        public static double TURN_DEAD_ZONE   = 0.05;
    }

    public static class LocalizerConstants {
        // Dead Wheels
        public static String FRONT_DEAD_WHEEL_NAME = "intakeMotor";
        public static String LEFT_DEAD_WHEEL_NAME  = DrivebaseConstants.BACK_LEFT_DRIVE_MOTOR_NAME;
        public static String RIGHT_DEAD_WHEEL_NAME = DrivebaseConstants.BACK_RIGHT_DRIVE_MOTOR_NAME;

        public static double TRACK_WIDTH_INCHES              = 11.2;
        public static double DEAD_WHEEL_DIAMETER_INCHES      = 1.88976;
        public static double DEAD_WHEEL_CIRCUMFERENCE_INCHES = Math.PI * DEAD_WHEEL_DIAMETER_INCHES;
        public static double ENCODER_TICKS_PER_REVOLUTION    = 2000.0;

        // Spark Fun
        public static String OPTICAL_ODOMETRY_NAME = "sparkFunOTOS";
        public static double LINEAR_SCALAR = 1.0;
        public static double ANGULAR_SCALAR = 1.0;
        public static int  IMU_CALIBRATION_SAMPLES = 255;
    }
}
