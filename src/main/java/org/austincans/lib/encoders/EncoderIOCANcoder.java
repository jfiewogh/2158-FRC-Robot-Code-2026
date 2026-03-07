package org.austincans.lib.encoders;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;

public class EncoderIOCANcoder implements EncoderIO {
    private final CANcoder encoder;

    public EncoderIOCANcoder(int deviceId, CANBus canBus) {
        encoder = new CANcoder(deviceId, canBus);
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.positionRad = getPositionRad();
        inputs.velocityRadPerSec = getVelocityRadPerSec();
    }

    @Override
    public double getPositionRad() {
        return encoder.getPosition().getValue().in(Radians);
    }

    @Override
    public double getVelocityRadPerSec() {
        return encoder.getVelocity().getValue().in(RadiansPerSecond);
    }
}
