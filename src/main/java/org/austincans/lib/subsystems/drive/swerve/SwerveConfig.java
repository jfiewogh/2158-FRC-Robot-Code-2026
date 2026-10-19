package org.austincans.lib.subsystems.drive.swerve;

import org.austincans.lib.io.motors.configs.PIDConfig;
import org.austincans.lib.subsystems.drive.DriveConfig;

public class SwerveConfig extends DriveConfig {
    public SwerveModuleConfig moduleConfig;
    public PIDConfig drivePIDConfig;
    public PIDConfig steerPIDConfig;
    public double odometryFrequencyHz;
}