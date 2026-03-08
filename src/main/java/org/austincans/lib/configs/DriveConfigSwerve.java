package org.austincans.lib.configs;

public class DriveConfigSwerve extends DriveConfig {
    public ModuleConfig moduleConfig;
    public PIDConfig drivePIDConfig;
    public PIDConfig steerPIDConfig;
    public double odometryFrequencyHz;
}
