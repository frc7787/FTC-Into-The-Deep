package org.firstinspires.ftc.teamcode.constants;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

public final class Constants {

    /// Contains hardcoded file locations that are commenly used
    public static class FileConstants {
        @SuppressLint("sdCardPath")
        public static final String CONSTANTS_FILE_LOCATION
            = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/Constants/";
    }

    /// Contains all drive base constants. Not to be confused with 
    //  {@link RoadrunnerConstants}
    public static class DrivebaseConstants {
        public static String FRONT_LEFT_DRIVE_MOTOR_NAME  = "frontLeftDriveMotor";
        public static String FRONT_RIGHT_DRIVE_MOTOR_NAME = "frontRightDriveMotor";
        public static String BACK_LEFT_DRIVE_MOTOR_NAME   = "backLeftDriveMotor";
        public static String BACK_RIGHT_DRIVE_MOTOR_NAME  = "backRightDriveMotor";

        public static double DRIVE_DEAD_ZONE  = 0.05;
        public static double STRAFE_DEAD_ZONE = 0.05;
        public static double TURN_DEAD_ZONE   = 0.05;
    }

    public static class ServoTestConstants {
        public static String TEST_SERVO_NAME               = "TestServo";
        public static double TEST_SERVO_START_POSITION     = 0.00;
        public static Servo.Direction TEST_SERVO_DIRECTION = Servo.Direction.FORWARD;
    }

    public static class RoadrunnerConstants {
        public static String IMU_NAME = "imu";

        public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION
                = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION
                = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        public static RevHubOrientationOnRobot IMU_ORIENTATION =
            new RevHubOrientationOnRobot(LOGO_FACING_DIRECTION, USB_FACING_DIRECTION);

        // Drive Model Parameters
        public static double INCHES_PER_TICK = 1;
        public static double LATERAL_INCHES_PER_TICK= INCHES_PER_TICK;
        public static double TRACK_WIDTH_TICKS = 0;

        // Feedforward Parameters (in tick units)
        public static double KS = 0;
        public static double KV = 0;
        public static double KA = 0;

        // Path Profile Parameters (inches/s)
        public static double MAX_WHEEL_VELOCITY       = 50;
        public static double MIN_PROFILE_ACCELERATION = -30;
        public static double MAX_PROFILE_ACCELERATION = 50;

        // Turn Profile Parameters (Radians)
        public static double MAX_ANGULAR_VELOCITY_RADIANS = Math.PI;
        public static double MAX_ANGULAR_ACCELERATION     = Math.PI;

        // Path Controller Gains
        public static double AXIAL_GAIN   = 0.0;
        public static double LATERAL_GAIN = 0.0;
        public static double HEADING_GAIN = 0.0;

        public static double AXIAL_VELOCITY_GAIN   = 0.0;
        public static double LATERAL_VELOCITY_GAIN = 0.0;
        public static double HEADING_VELOCITY_GAIN = 0.0;
    }
}
