package org.austincans.frc2026.subsystems.feeder;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IndexerCommands {
    private IndexerCommands() {}

    public static Command runCommand(Optional<Indexer> indexer) {
        return indexer.map((i) -> Commands.runOnce(i::run, i)).orElse(Commands.none());
    }

    public static Command stopCommand(Optional<Indexer> indexer) {
        return indexer.map((i) -> Commands.runOnce(i::stop, i)).orElse(Commands.none());
    }

    public static boolean atSetpoint(Optional<Indexer> indexer) {
        return indexer.map((i) -> i.atSetpoint()).orElse(true);
    }
}
