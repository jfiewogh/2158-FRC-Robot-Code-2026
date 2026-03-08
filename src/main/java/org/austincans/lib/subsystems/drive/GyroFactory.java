package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.GyroConfigPigeon2;

public class GyroFactory {
    public GyroIOPigeon2 createPigeon2(GyroConfigPigeon2 gyroConfig) {
        return new GyroIOPigeon2(gyroConfig, new Pigeon2Configuration());
    }
}
