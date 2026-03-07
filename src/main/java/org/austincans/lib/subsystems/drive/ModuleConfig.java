package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.PIDFFConfig;

public record ModuleConfig(
    double drivekP,
    double drivekI,
    double drivekD,
    double drivekS,
    double drivekV,
    double drivekT,
    double steerkP,
    double steerkI,
    double steerkD,
    double steerkS,
    double steerkV,
    double steerkA,
    double maxDriveMotorVelocityRadPerSec,
    double maxSteerMotorVelocityRadPerSec
) {
    public PIDFFConfig drivePIDFFConfig() {
        return new PIDFFConfig(drivekP, drivekI, drivekD, drivekS, 0, drivekV, 0, drivekT);
    }
    public PIDFFConfig steerPIDFFConfig() {
        return new PIDFFConfig(steerkP, steerkI, steerkD, steerkS, 0, steerkV, steerkA, 0);
    } 
}
