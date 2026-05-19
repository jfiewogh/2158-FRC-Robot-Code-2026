package org.austincans.lib.hardware.sensors.gyros;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

public class GyroFactory {
    public static GyroIOPigeon2 createPigeon2(GyroConfigPigeon2 gyroConfig) {
        return new GyroIOPigeon2(gyroConfig, new Pigeon2Configuration());
    }
}
