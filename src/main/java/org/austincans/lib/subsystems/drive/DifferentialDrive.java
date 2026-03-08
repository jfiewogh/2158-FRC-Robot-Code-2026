package org.austincans.lib.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.controllers.PPLTVController;

public class DifferentialDrive extends DriveBase {
    public DifferentialDrive(String name, DriveIO io, PathPlannerConfig pathPlannerConfig) {
        super(name, io, new PPLTVController(
            new PIDConstants(),
            new PIDConstants()));
    }
    
    public void arcadeDrive(DoubleSupplier xInput, DoubleSupplier omegaInput) {
        runVelocities(xInput.getAsDouble(), 0, omegaInput.getAsDouble());
    }
}
