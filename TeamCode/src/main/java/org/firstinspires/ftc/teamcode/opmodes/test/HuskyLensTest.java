package org.firstinspires.ftc.teamcode.opmodes.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.drivers.CustomHuskyLens;

public class HuskyLensTest extends CommandOpMode {
    private CustomHuskyLens huskyLens;


    @Override public void initialize() {
       huskyLens = hardwareMap.get(CustomHuskyLens.class, "huskyLens");

       if (!huskyLens.knock()) telemetry.addLine("Problem communicating with husky lens");

       huskyLens.selectAlgorithm(CustomHuskyLens.Algorithm.COLOR_RECOGNITION);

       schedule(
               new RunCommand(this::readBlocks),
               new RunCommand(telemetry::update)
       );
    }

    private void readBlocks() {
        CustomHuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        for (CustomHuskyLens.Block block : blocks) {
            telemetry.addData("Block", block.toString());
        }
        telemetry.update();
    }
}
