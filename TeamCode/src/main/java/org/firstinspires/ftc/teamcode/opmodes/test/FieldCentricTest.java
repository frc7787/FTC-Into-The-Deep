package org.firstinspires.ftc.teamcode.opmodes.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.commands.FieldCentricCommand;

@TeleOp(name = "Test - Field Centric")
public class FieldCentricTest extends CommandOpMode {

    @Override public void initialize() {
        MecanumDriveSubsystem driveSubsystem = new MecanumDriveSubsystem(this);
        GamepadEx driverGamepad = new GamepadEx(gamepad1);

        register(driveSubsystem);

        schedule(
                new FieldCentricCommand(
                        driveSubsystem,
                        driverGamepad::getLeftX,
                        driverGamepad::getLeftY,
                        driverGamepad::getRightY
                )
        );
    }
}
