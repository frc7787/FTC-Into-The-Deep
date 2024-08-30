package org.firstinspires.ftc.teamcode.opmodes.test;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.commands.RobotCentricCommand;

@TeleOp(name = "Test - Robot Centric", group = "Test")
public class RobotCentricTest extends CommandOpMode {

    @Override public void initialize() {
        DriveSubsystem driveSubsystem = new DriveSubsystem(this);
        GamepadEx driverGamepad       = new GamepadEx(gamepad1);

        register(driveSubsystem);

        schedule(
                new RobotCentricCommand(
                        driveSubsystem,
                        driverGamepad::getLeftX,
                        driverGamepad::getLeftY,
                        driverGamepad::getRightX
                ),
                new RunCommand(driveSubsystem::debug),
                new RunCommand(telemetry::update)
        );
    }
}
