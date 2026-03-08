package org.austincans.lib.subsystems.drive;

import java.util.function.DoubleSupplier;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

public class DifferentialDrive<T extends DriveIO> extends DriveBase {
    protected DifferentialDrive(DriveIO io, PathPlannerConfig pathPlannerConfig) {
        PPLTVController pathFollowerController = new PPLTVController(
            new PIDConstants(), // get values from pathplannerconfig
            new PIDConstants())); // get values from pathplannerconfig
        RobotConfig robotConfig; // get values from pathplannerconfig
        super(io, pathFollowerController, robotConfig);
    }
    
    public void arcadeDrive(DoubleSupplier xInput, DoubleSupplier omegaInput) {
        runVelocities(xInput.getAsDouble(), 0, omegaInput.getAsDouble());
    }
}
