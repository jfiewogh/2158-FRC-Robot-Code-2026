package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.PIDFFConfig;

public record DriveConfig(
    double trackWidthMeters, // distance between left and right motors
    double wheelbaseMeters, // distance between front and back motors
    double drivekP,
    double drivekI,
    double drivekD,
    double drivekS,
    double drivekV,
    double driveKT,
    double steerkP,
    double steerkI,
    double steerkD,
    double steerkS,
    double steerkV,
    double steerkA
) {
    public PIDFFConfig drivePIDFFConfig() {
        return new PIDFFConfig(drivekP, drivekI, steerkD, drivekS, 0, drivekV, 0, driveKT);
    }
    public PIDFFConfig steerPIDFFConfig() {
        return new PIDFFConfig(steerkP, steerkI, steerkD, steerkS, 0, steerkV, steerkA, 0);
    } 
}
