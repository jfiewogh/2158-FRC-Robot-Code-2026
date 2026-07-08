package org.austincans.lib.subsystems.drive.differential;

import org.austincans.lib.io.motors.MotorConfig;
import org.austincans.lib.subsystems.drive.DriveConfig;

public class DifferentialDriveConfig<MotorConfigType extends MotorConfig> extends DriveConfig {
    public MotorConfigType leftLeaderMotorConfig;
    public MotorConfigType rightLeaderMotorConfig;
}
