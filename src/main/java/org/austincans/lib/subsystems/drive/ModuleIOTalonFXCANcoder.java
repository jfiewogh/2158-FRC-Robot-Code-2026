package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.ModuleConfig;
import org.austincans.lib.encoders.AbsoluteEncoderFactory;
import org.austincans.lib.encoders.CANcoderIO;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOTalonFX;

public class ModuleIOTalonFXCANcoder extends ModuleIOBase<MotorIOTalonFX, MotorIOTalonFX, CANcoderIO> {
    public ModuleIOTalonFXCANcoder(ModuleConfig moduleConfig) {
        super(
            MotorFactory.createTalonFX(moduleConfig.driveMotorConfig()),
            MotorFactory.createTalonFX(moduleConfig.steerMotorConfig()),
            AbsoluteEncoderFactory.createCANcoder(moduleConfig.absoluteEncoderConfig())
        );
    }
}
