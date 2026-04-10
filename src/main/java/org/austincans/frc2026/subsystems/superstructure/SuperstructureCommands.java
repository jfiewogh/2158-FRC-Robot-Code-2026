package org.austincans.frc2026.subsystems.superstructure;

import static edu.wpi.first.units.Units.RPM;

import org.austincans.frc2026.subsystems.feeder.Indexer;
import org.austincans.frc2026.subsystems.feeder.Spindexer;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;

/** Commands that require more than one subsystem */
public class SuperstructureCommands {
    private SuperstructureCommands() {}

    public void runFeederAndFlywheel(Flywheel flywheel, Indexer indexer, Spindexer spindexer, double flywheelVelocityRPM) {
        flywheel.setVelocity(RPM.of(flywheelVelocityRPM));
        indexer.run();
        if (flywheel.atSetpoint() && indexer.atSetpoint()) {
            spindexer.run();
        }
    }

    public void stopFeederAndFlywheel(Flywheel flywheel, Indexer indexer, Spindexer spindexer) {
        flywheel.stopShooter();
        indexer.stop();
        spindexer.stop();
    }
}
