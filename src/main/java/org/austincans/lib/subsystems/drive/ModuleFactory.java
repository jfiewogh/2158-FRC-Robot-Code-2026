package org.austincans.lib.subsystems.drive;

import org.austincans.lib.subsystems.drive.ModuleIOTalonFXCANcoder;

public class ModuleFactory {
    public static ModuleIOTalonFXCANcoder createTalonFXCANCoder(ModuleConfig moduleConfig) {
        return new ModuleIOTalonFXCANcoder(
            
        );
    }
}
