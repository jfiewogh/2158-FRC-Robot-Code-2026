package org.austincans.lib.subsystems.drive;

public class ModuleFactory {
    public static ModuleIOTalonFXCANcoder createTalonFXCANCoder(ModuleConfig moduleConfig) {
        return new ModuleIOTalonFXCANcoder(
            
        );
    }
}
