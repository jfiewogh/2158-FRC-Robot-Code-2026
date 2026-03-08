package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.ModuleConfig;

public class ModuleFactory {
    public static ModuleIOTalonFXCANcoder createTalonFXCANCoder(com.pathplanner.lib.config.ModuleConfig moduleConfig) {
        return new ModuleIOTalonFXCANcoder(
            
        );
    }
}
