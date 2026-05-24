package org.austincans.lib.subsystems.drive;

import org.austincans.lib.io.motors.MotorConfig;
import org.austincans.lib.io.sensors.encoders.AbsoluteEncoderConfig;

public class SwerveModuleConfig<DriveMotorConfigType extends MotorConfig, SteerMotorConfigType extends MotorConfig, AbsoluteEncoderConfigType extends AbsoluteEncoderConfig> {
    public DriveMotorConfigType driveMotorConfig;
    public SteerMotorConfigType steerMotorConfig;
    public AbsoluteEncoderConfigType absoluteEncoderConfig;
    /** The drive speed multiplier on initialization */
    public double driveSpeedMultiplier;
}
