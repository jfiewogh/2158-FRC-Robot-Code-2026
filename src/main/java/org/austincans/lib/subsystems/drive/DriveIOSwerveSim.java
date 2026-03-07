package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.MotorIOSim;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Mass;

public class DriveIOSwerveSim extends DriveIOSwerve<MotorIOSim, GyroIOSim> {
    private final SwerveDriveSimulation simulation;

    public DriveIOSwerveSim(DriveConfig driveConfig) {
        super(driveConfig, ModuleIOSim());
        simulation = new SwerveDriveSimulation(
            new DriveTrainSimulationConfig(
                Mass.ofBaseUnits(driveConfig.robotMassKg(), Units.Kilogram), 
                null, 
                null, 
                null, 
                null, 
                null, 
                null), 
                null);
    }

    @Override
    public void updateInputs() {

    }
}
