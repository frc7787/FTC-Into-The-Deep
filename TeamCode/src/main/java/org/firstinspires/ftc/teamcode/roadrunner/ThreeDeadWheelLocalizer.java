package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.messages.ThreeDeadWheelInputsMessage;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double leftDeadWheelYTicks  = 0.0; // y position of the first parallel encoder (in tick units)
        public double rightDeadWheelYTicks = 1.0; // y position of the second parallel encoder (in tick units)
        public double frontDeadWheelXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder leftDeadWheel, rightDeadWheel, frontDeadWheel;

    public final double inPerTick;

    private int prevLeftDeadWheelPos, prevRightDeadWheelPos, prevFrontDeadWheelPos;
    private boolean initialized;

    public ThreeDeadWheelLocalizer(@NonNull HardwareMap hardwareMap, double inPerTick) {
        leftDeadWheel  = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "BackLeftDriveMotor")));
        rightDeadWheel = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "BackRightDriveMotor")));
        frontDeadWheel = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "IntakeMotor")));

        leftDeadWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frontDeadWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair leftDeadWheelPosAndVelocity  = leftDeadWheel.getPositionAndVelocity();
        PositionVelocityPair rightDeadWheelPosAndVelocity = rightDeadWheel.getPositionAndVelocity();
        PositionVelocityPair frontDeadWheelPosAndVelocity = frontDeadWheel.getPositionAndVelocity();

        FlightRecorder.write(
                "THREE_DEAD_WHEEL_INPUTS",
                new ThreeDeadWheelInputsMessage(
                        leftDeadWheelPosAndVelocity,
                        rightDeadWheelPosAndVelocity,
                        frontDeadWheelPosAndVelocity));

        if (!initialized) {
            initialized = true;

            prevLeftDeadWheelPos = leftDeadWheelPosAndVelocity.position;
            prevRightDeadWheelPos = rightDeadWheelPosAndVelocity.position;
            prevFrontDeadWheelPos = frontDeadWheelPosAndVelocity.position;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int leftDeadWheelDelta  = leftDeadWheelPosAndVelocity.position - prevLeftDeadWheelPos;
        int rightDeadWheelDelta = rightDeadWheelPosAndVelocity.position - prevRightDeadWheelPos;
        int frontDeadWheelDelta = frontDeadWheelPosAndVelocity.position - prevFrontDeadWheelPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.leftDeadWheelYTicks * rightDeadWheelDelta - PARAMS.rightDeadWheelYTicks * leftDeadWheelDelta) / (PARAMS.leftDeadWheelYTicks - PARAMS.rightDeadWheelYTicks),
                                (PARAMS.leftDeadWheelYTicks * rightDeadWheelPosAndVelocity.velocity - PARAMS.rightDeadWheelYTicks * leftDeadWheelPosAndVelocity.velocity) / (PARAMS.leftDeadWheelYTicks - PARAMS.rightDeadWheelYTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.frontDeadWheelXTicks / (PARAMS.leftDeadWheelYTicks - PARAMS.rightDeadWheelYTicks) * (rightDeadWheelDelta - leftDeadWheelDelta) + frontDeadWheelDelta),
                                (PARAMS.frontDeadWheelXTicks / (PARAMS.leftDeadWheelYTicks - PARAMS.rightDeadWheelYTicks) * (rightDeadWheelPosAndVelocity.velocity - leftDeadWheelPosAndVelocity.velocity) + frontDeadWheelPosAndVelocity.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (leftDeadWheelDelta - rightDeadWheelDelta) / (PARAMS.leftDeadWheelYTicks - PARAMS.rightDeadWheelYTicks),
                        (leftDeadWheelPosAndVelocity.velocity - rightDeadWheelPosAndVelocity.velocity) / (PARAMS.leftDeadWheelYTicks - PARAMS.rightDeadWheelYTicks),
                })
        );

        prevLeftDeadWheelPos = leftDeadWheelPosAndVelocity.position;
        prevRightDeadWheelPos = rightDeadWheelPosAndVelocity.position;
        prevFrontDeadWheelPos = frontDeadWheelPosAndVelocity.position;

        return twist;
    }
}
