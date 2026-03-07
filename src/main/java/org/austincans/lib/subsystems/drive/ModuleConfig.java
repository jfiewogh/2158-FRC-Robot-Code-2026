package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.PIDFFConfig;

public record ModuleConfig(
    double drivekP,
    double drivekI,
    double drivekD,
    double driveKS,
    double drivekV,
    double driveKT,
    double steerkP,
    double steerkD
) {
    public PIDFFConfig drivePIDFFConfig() {
        return new PIDFFConfig(drivekP, drivekI, steerkD, driveKS, 0, drivekV, 0, driveKT);
    }
    public PIDFFConfig steerPIDFFConfig() {
        return new PIDFFConfig(steerkP, 0, steerkD, 0, 0, 0, 0, 0);
    }
}
