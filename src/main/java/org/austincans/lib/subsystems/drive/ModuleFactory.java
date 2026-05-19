package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.MotorConfigTalon;
import org.austincans.lib.configs.SwerveModuleConfig;

public class ModuleFactory {
    public static ModuleIOTalonFXCANcoder createTalonFXCANCoder(SwerveModuleConfig<TalonConfig, TalonConfig> moduleConfig) {
        return new ModuleIOTalonFXCANcoder(moduleConfig);
    }
}
