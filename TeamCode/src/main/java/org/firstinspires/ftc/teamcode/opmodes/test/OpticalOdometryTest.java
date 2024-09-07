package org.firstinspires.ftc.teamcode.opmodes.test;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.*;

import org.firstinspires.ftc.teamcode.purepursuit.localization.OpticalLocalizer;

public class OpticalOdometryTest extends CommandOpMode {
    private OpticalLocalizer localizer;

    @Override public void initialize() {
        OpticalLocalizer localizer = new OpticalLocalizer(hardwareMap);

        configureBindings();

        schedule(
                new RunCommand(() -> localizer.debug(telemetry)),
                new RunCommand(telemetry::update)
        );
    }

    private void configureBindings() {
        GamepadEx driverGamepad   = new GamepadEx(gamepad1);
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);

        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .or(new GamepadButton(operatorGamepad, GamepadKeys.Button.B))
                .whenActive(localizer::reset);
    }

}
