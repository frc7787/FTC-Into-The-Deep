package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import org.firstinspires.ftc.teamcode.purepursuit.localization.DeadWheel;

@TeleOp(name = "Test - Dead Wheel", group = "Test")
public final class DeadWheelTest extends CommandOpMode {

    @Override public void initialize() {
        DeadWheel frontDeadWheel
                = new DeadWheel(hardwareMap.get(DcMotorImplEx.class, FRONT_DEAD_WHEEL_NAME));
        DeadWheel leftDeadWheel
                = new DeadWheel(hardwareMap.get(DcMotorImplEx.class, LEFT_DEAD_WHEEL_NAME));
        DeadWheel rightDeadWheel
                = new DeadWheel(hardwareMap.get(DcMotorImplEx.class, RIGHT_DEAD_WHEEL_NAME));

        leftDeadWheel.reverse();

        schedule(
                new RunCommand(() -> frontDeadWheel.debug(telemetry)),
                new RunCommand(() -> leftDeadWheel.debug(telemetry)),
                new RunCommand(() -> rightDeadWheel.debug(telemetry)),
                new RunCommand(telemetry::update)
        );
    }
}
