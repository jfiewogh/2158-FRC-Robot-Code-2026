package org.austincans.lib.encoders;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;

public class CANcoderIO implements AbsoluteEncoderIO {
    private final CANcoder encoder;

    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    public CANcoderIO(AbsoluteEncoderConfig absoluteEncoderConfig, CANcoderConfiguration config, Frequency positionUpdateFrequency, boolean isCanivore) {
        encoder = new CANcoder(absoluteEncoderConfig.deviceId(), new CANBus(absoluteEncoderConfig.canbus()));
        encoder.getConfigurator().apply(config);
        absolutePosition = encoder.getAbsolutePosition();
        position = encoder.getPosition(); // relative to start
        velocity = encoder.getVelocity();
    }

    @Override
    public void updateInputs(AbsoluteEncoderIOInputs inputs) {
        inputs.connected = encoder.isConnected();
        inputs.absolutePositionRad = absolutePosition.getValue().in(Radians);
        inputs.positionRad = position.getValue().in(Radians);
        inputs.velocityRadPerSec = velocity.getValue().in(RadiansPerSecond);
    }

    public int getDeviceId() {
        return encoder.getDeviceID();
    }
}
