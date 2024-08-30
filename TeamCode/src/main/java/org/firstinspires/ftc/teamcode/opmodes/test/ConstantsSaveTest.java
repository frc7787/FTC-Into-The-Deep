package org.firstinspires.ftc.teamcode.opmodes.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ConstantsSaver;

@TeleOp(name = "Test - Constants Save", group = "Test")
public class ConstantsSaveTest extends CommandOpMode {

    @Override public void initialize() {
       new ConstantsSaver(telemetry).save();
    }
}
