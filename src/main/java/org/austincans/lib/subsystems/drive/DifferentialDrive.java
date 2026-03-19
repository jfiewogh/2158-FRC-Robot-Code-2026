package org.austincans.lib.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.austincans.lib.subsystems.drive.DifferentialDriveIO.DifferentialDriveIORecords;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

public class DifferentialDrive {
    private final DifferentialDriveIO io;
    private final DifferentialDriveIORecordsAutoLogged records = new DifferentialDriveIORecordsAutoLogged();
    private final DifferentialDriveIORequestsAutoLogged requests = new DifferentialDriveIORequestsAutoLogged();

    protected DifferentialDrive(DifferentialDriveIO io, PPLTVController pathFollowerController, RobotConfig robotConfig) {
        this.io = io;
    }

    public void arcadeDrive(DoubleSupplier xInput, DoubleSupplier omegaInput) {
        io.runVelocities(xInput.getAsDouble(), 0, omegaInput.getAsDouble());
    }
}
