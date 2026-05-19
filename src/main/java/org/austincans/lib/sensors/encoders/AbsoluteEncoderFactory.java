package org.austincans.lib.sensors.encoders;

import org.austincans.lib.configs.AbsoluteEncoderConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class AbsoluteEncoderFactory {
    public static CANcoderIO createCANcoder(AbsoluteEncoderConfig absoluteEncoderConfig) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = absoluteEncoderConfig.offsetRotations();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = absoluteEncoderConfig.discontinuityPoint();
        config.MagnetSensor.SensorDirection = absoluteEncoderConfig.isInverted() 
            ? SensorDirectionValue.CounterClockwise_Positive 
            : SensorDirectionValue.Clockwise_Positive;
        return new CANcoderIO(absoluteEncoderConfig.deviceId(), new CANBus(absoluteEncoderConfig.canbus()), config);
    }
}
