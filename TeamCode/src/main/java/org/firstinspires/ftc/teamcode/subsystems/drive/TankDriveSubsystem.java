package org.firstinspires.ftc.teamcode.subsystems.drive;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.utility.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.Range;

public class TankDriveSubsystem extends SubsystemBase {
    MotorGroup leftMotorGroup, rightMotorGroup;

    public TankDriveSubsystem(@NonNull OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        DcMotorImplEx leftMotorOne = hardwareMap.get(DcMotorImplEx.class, "leftMotorOne");
        DcMotorImplEx rightMotorOne = hardwareMap.get(DcMotorImplEx.class, "rightMotorOne");
        DcMotorImplEx leftMotorTwo = hardwareMap.get(DcMotorImplEx.class, "leftMotorTwo");
        DcMotorImplEx rightMotorTwo = hardwareMap.get(DcMotorImplEx.class, "rightMotorTwo");

        leftMotorGroup = new MotorGroup(Direction.FORWARD, leftMotorOne, leftMotorTwo);
        rightMotorGroup = new MotorGroup(Direction.REVERSE, rightMotorOne, rightMotorTwo);
    }

    public void duelStickArcade(double leftPower, double rightPower) {
        leftMotorGroup.setPower(leftPower);
        rightMotorGroup.setPower(rightPower);
    }

    public void singleStickArcade(double drivePower, double turnPower) {
        double leftPower = Range.clip(drivePower + turnPower, -1, 1);
        double rightPower = Range.clip(drivePower - turnPower, -1, 1);

        leftMotorGroup.setPower(leftPower);
        rightMotorGroup.setPower(rightPower);
    }
}
