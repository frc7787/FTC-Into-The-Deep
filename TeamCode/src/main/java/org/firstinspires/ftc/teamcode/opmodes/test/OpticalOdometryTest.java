package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.utility.playstationcontroller.PlayStationController.*;

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
                new RunCommand(
                        () -> telemetry.addLine("Press Options On Either Controller To Reset")
                ),
                new RunCommand(telemetry::update)
        );
    }

    private void configureBindings() {
        GamepadEx driverGamepad   = new GamepadEx(gamepad1);
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);

        new GamepadButton(driverGamepad, OPTIONS)
                .or(new GamepadButton(operatorGamepad, OPTIONS))
                .whenActive(localizer::reset);
    }

}
