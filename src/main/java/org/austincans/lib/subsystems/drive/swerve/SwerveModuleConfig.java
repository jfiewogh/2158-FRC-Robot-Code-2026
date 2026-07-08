package org.austincans.lib.subsystems.drive.swerve;

import org.austincans.lib.io.Config;

public class SwerveModuleConfig {
    public Config driveMotorConfig;
    public Config steerMotorConfig;
    public Config absoluteEncoderConfig;
    /** The drive speed multiplier on initialization */
    public double driveSpeedMultiplier;
}
