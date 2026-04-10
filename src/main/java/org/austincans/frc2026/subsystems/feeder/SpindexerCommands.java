package org.austincans.frc2026.subsystems.feeder;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SpindexerCommands {
    private SpindexerCommands() {}

    public static Command runCommand(Optional<Spindexer> spindexer) {
        return spindexer.map((s) -> Commands.runOnce(s::run, s)).orElse(Commands.none());
    }

    public static Command stopCommand(Optional<Spindexer> spindexer) {
        return spindexer.map((s) -> Commands.runOnce(s::stop, s)).orElse(Commands.none());
    }
}