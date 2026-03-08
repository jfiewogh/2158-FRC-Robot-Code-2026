package org.austincans.lib.subsystems.drive;

import java.util.function.DoubleSupplier;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

public class DifferentialDrive<T extends DriveIODifferential> extends DriveBase {
    protected DifferentialDrive(T io, PPLTVController pathFollowerController, RobotConfig robotConfig) {
        super(io, pathFollowerController, robotConfig);
    }

    public void arcadeDrive(DoubleSupplier xInput, DoubleSupplier omegaInput) {
        runVelocities(xInput.getAsDouble(), 0, omegaInput.getAsDouble());
    }
}
