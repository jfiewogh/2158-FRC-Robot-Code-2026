package org.austincans.lib.configs;

import org.austincans.lib.util.swerve.ModuleLimits;

public class SwerveModuleConfig<DriveMotorCfg extends MotorConfig, SteerMotorCfg extends MotorConfig> {
    public DriveMotorCfg driveMotorConfig;
    public SteerMotorCfg steerMotorConfig;
    public AbsoluteEncoderConfig absoluteEncoderConfig;
    public double wheelRadiusMeters;
    public double wheelCoefficientOfFriction;
    public double driveSpeedMultiplier;

    public ModuleLimits getModuleLimitsFree() {
        return new ModuleLimits(
            driveMotorConfig.maxSpeedRotationsPerSec,
            driveMotorConfig.maxAccelerationRotationsPerSecSq,
            steerMotorConfig.maxAccelerationRotationsPerSecSq);
    }
}
