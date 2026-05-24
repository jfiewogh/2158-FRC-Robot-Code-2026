package org.austincans.lib.io.sensors.encoders;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CANcoderIO implements AbsoluteEncoderIO {
    private final CANcoder encoder;

    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    protected CANcoderIO(int deviceId, CANBus canbus, CANcoderConfiguration config) {
        encoder = new CANcoder(deviceId, canbus);
        encoder.getConfigurator().apply(config);
        absolutePosition = encoder.getAbsolutePosition();
        position = encoder.getPosition(); // relative to start
        velocity = encoder.getVelocity();
    }

    @Override
    public void updateInputs(AbsoluteEncoderIOInputs inputs) {
        inputs.connected = encoder.isConnected();
        inputs.absolutePosition = absolutePosition.getValue();
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
    }

    public int getDeviceId() {
        return encoder.getDeviceID();
    }

    public CANcoder getEncoder(){
        return encoder;
    }
}
