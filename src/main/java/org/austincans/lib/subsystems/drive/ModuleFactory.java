package org.austincans.lib.subsystems.drive;

public class ModuleFactory {
    public static void createTalonFXCANCoder(ModuleConfig moduleConfig) {
        return new ModuleIOTalonFXCANcoder(

        );
    }
}
