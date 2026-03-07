package org.austincans.lib.subsystems.drive;

import java.util.function.DoubleSupplier;

public class DifferentialDrive extends DriveBase {
    public DifferentialDrive(String name, DriveIO io) {
        super(name, io);
    }
    
    public void arcadeDrive(DoubleSupplier xInput, DoubleSupplier omegaInput) {
        runVelocities(xInput.getAsDouble(), 0, omegaInput.getAsDouble());
    }
}
