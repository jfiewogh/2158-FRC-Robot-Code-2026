package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.PIDFFConfig;
import org.austincans.lib.util.swerve.ModuleLimits;

public record ModuleConfig(
    PIDFFConfig drivePIDFFConfig,
    PIDFFConfig steerPIDFFConfig,
    double maxDriveMotorVelocityRadPerSec,
    double maxDriveMotorAccelerationRadPerSecSq,
    double maxSteerMotorVelocityRadPerSec
    double wheelRadius) {
    public ModuleLimits moduleLimitsFree() {
        return new ModuleLimits(
            maxDriveMotorVelocityRadPerSec,
            maxDriveMotorAccelerationRadPerSecSq,
            maxSteerMotorVelocityRadPerSec);
    }
}
