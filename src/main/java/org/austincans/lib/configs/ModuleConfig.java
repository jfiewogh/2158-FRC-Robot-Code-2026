package org.austincans.lib.configs;

import org.austincans.lib.util.swerve.ModuleLimits;

public record ModuleConfig(
    MotorConfig driveMotorConfig,
    MotorConfig steerMotorConfig,
    AbsoluteEncoderConfig absoluteEncoderConfig,
    double wheelRadiusMeters,
    double driveSpeedMultiplier) {
    public ModuleLimits moduleLimitsFree() {
        return new ModuleLimits(
            driveMotorConfig.maxSpeedRotationsPerSec(),
            driveMotorConfig.maxAccelerationRotationsPerSecSq(),
            steerMotorConfig.maxAccelerationRotationsPerSecSq());
    }
}
