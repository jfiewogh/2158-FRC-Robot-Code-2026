package org.austincans.lib.configs;

public class DifferentialDriveConfig<MotorCnfg extends MotorConfig> extends DriveConfig {
    public MotorCnfg leftLeaderMotorConfig;
    public MotorCnfg rightLeaderMotorConfig;
}
