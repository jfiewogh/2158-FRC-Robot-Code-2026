package org.austincans.lib.subsystems.drive;

import org.austincans.lib.io.motors.MotorConfig;

public class DifferentialDriveConfig<MotorConfigType extends MotorConfig> extends DriveConfig {
    public MotorConfigType leftLeaderMotorConfig;
    public MotorConfigType rightLeaderMotorConfig;
}
