package org.austincans.lib.subsystems.drive.swerve;

import org.austincans.lib.configs.MotorConfigTalon;
import org.austincans.lib.configs.SwerveModuleConfig;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.TalonFXIO;
import org.austincans.lib.sensors.encoders.AbsoluteEncoderFactory;
import org.austincans.lib.sensors.encoders.CANcoderIO;

public class ModuleIOTalonFXCANcoder extends ModuleIOBase<TalonFXIO, TalonFXIO, CANcoderIO> {
    public ModuleIOTalonFXCANcoder(SwerveModuleConfig<TalonConfig, TalonConfig> moduleConfig) {
        super(
            MotorFactory.createTalonFX(moduleConfig.driveMotorConfig),
            MotorFactory.createTalonFX(moduleConfig.steerMotorConfig),
            AbsoluteEncoderFactory.createCANcoder(moduleConfig.absoluteEncoderConfig)
        );
    }
}
