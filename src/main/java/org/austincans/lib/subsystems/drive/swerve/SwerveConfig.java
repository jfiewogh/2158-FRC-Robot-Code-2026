package org.austincans.lib.subsystems.drive.swerve;

import org.austincans.lib.io.motors.MotorConfig;
import org.austincans.lib.io.motors.MotorConfig.PIDConfig;
import org.austincans.lib.io.sensors.encoders.AbsoluteEncoderConfig;
import org.austincans.lib.io.sensors.gyros.GyroConfig;
import org.austincans.lib.subsystems.drive.DriveConfig;

public class SwerveConfig<DriveMotorConfigType extends MotorConfig, SteerMotorConfigType extends MotorConfig, AbsoluteEncoderConfigType extends AbsoluteEncoderConfig, GyroConfigType extends GyroConfig> extends DriveConfig {
    public SwerveModuleConfig<DriveMotorConfigType, SteerMotorConfigType, AbsoluteEncoderConfigType> moduleConfig;
    public GyroConfigType gyroConfig;
    public PIDConfig drivePIDConfig;
    public PIDConfig steerPIDConfig;
    public double odometryFrequencyHz;
}