package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.FFConfig;
import org.austincans.lib.configs.PIDConfig;
import org.austincans.lib.util.swerve.ModuleLimits;

public record ModuleConfig(
    PIDConfig drivePIDConfig,
    FFConfig driveFFConfig,
    PIDConfig steerPIDConfig,
    FFConfig steerFFConfig,
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
