package org.austincans.frc2026.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class FlywheelCommands {
    private FlywheelCommands() {}

    public static Command runCommand(Optional<Flywheel> flywheel, double velocityRPM) {
        return flywheel.map((f) -> Commands.runOnce(() -> f.setVelocity(RPM.of(velocityRPM)), f)).orElse(Commands.none());
    }

    public static boolean atSetpoint(Optional<Flywheel> flywheel) {
        return flywheel.map((f) -> f.atSetpoint()).orElse(true);
    }
}
