package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.MotorConfigTalon;
import org.austincans.lib.configs.SwerveModuleConfig;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOTalonFX;
import org.austincans.lib.sensors.encoders.AbsoluteEncoderFactory;
import org.austincans.lib.sensors.encoders.CANcoderIO;

public class ModuleIOTalonFXCANcoder extends ModuleIOBase<MotorIOTalonFX, MotorIOTalonFX, CANcoderIO> {
    public ModuleIOTalonFXCANcoder(SwerveModuleConfig<MotorConfigTalon, MotorConfigTalon> moduleConfig) {
        super(
            MotorFactory.createTalonFX(moduleConfig.driveMotorConfig),
            MotorFactory.createTalonFX(moduleConfig.steerMotorConfig),
            AbsoluteEncoderFactory.createCANcoder(moduleConfig.absoluteEncoderConfig)
        );
    }
}
