package org.austincans.lib.configs;

import org.austincans.lib.util.swerve.ModuleLimits;

public record ModuleConfig(
    PIDConfig drivePIDConfig,
    FFConfig driveFFConfig,
    PIDConfig steerPIDConfig,
    FFConfig steerFFConfig,
    double maxDriveMotorVelocityRadPerSec,
    double maxDriveMotorAccelerationRadPerSecSq,
    double maxSteerMotorVelocityRadPerSec,
    double wheelRadiusMeters) {
    public ModuleLimits moduleLimitsFree() {
        return new ModuleLimits(
            maxDriveMotorVelocityRadPerSec,
            maxDriveMotorAccelerationRadPerSecSq,
            maxSteerMotorVelocityRadPerSec);
    }
}
