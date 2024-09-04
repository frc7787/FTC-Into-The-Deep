package org.firstinspires.ftc.teamcode.opmodes.test;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.commands.RobotCentricCommand;

@TeleOp(name = "Test - Robot Centric", group = "Test")
public final class RobotCentricTest extends CommandOpMode {
    GamepadEx driverGamepad;

    @Override public void initialize() {
        DriveSubsystem driveSubsystem = new DriveSubsystem(this);
        driverGamepad       = new GamepadEx(gamepad1);

        register(driveSubsystem);

        schedule(
                new RobotCentricCommand(
                        driveSubsystem,
                        driverGamepad::getLeftY,
                        driverGamepad::getLeftX,
                        driverGamepad::getRightX
                )
        );
    }
}
