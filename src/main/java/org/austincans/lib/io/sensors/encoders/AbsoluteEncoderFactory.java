package org.austincans.lib.io.sensors.encoders;

import com.ctre.phoenix6.CANBus;

public class AbsoluteEncoderFactory {
    public static CANcoderIO createCANcoder(int deviceId, CANBus canBus) {
        return new CANcoderIO(deviceId, canBus);
    }
}
