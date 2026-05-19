package org.austincans.lib.configs;

// TODO: should extend ModuleConfig
public class SwerveConfig<DriveMtrCnfg extends MotorConfig, SteerMtrCnfg extends MotorConfig, GyroCnfg extends GyroConfig> extends DriveConfig {
    public SwerveModuleConfig<DriveMtrCnfg, SteerMtrCnfg> moduleConfig;
    public GyroCnfg gyroConfig;
    public PIDConfig drivePIDConfig;
    public PIDConfig steerPIDConfig;
    public double odometryFrequencyHz;
}