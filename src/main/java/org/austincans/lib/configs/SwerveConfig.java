package org.austincans.lib.configs;

public class SwerveConfig<DriveMotorCfg extends MotorConfig, SteerMotorCfg extends MotorConfig, GyroCfg extends GyroConfig> extends DriveConfig {
    public SwerveModuleConfig<DriveMotorCfg, SteerMotorCfg> moduleConfig;
    public GyroCfg gyroConfig;
    public PIDConfig drivePIDConfig;
    public PIDConfig steerPIDConfig;
    public double odometryFrequencyHz;
}
