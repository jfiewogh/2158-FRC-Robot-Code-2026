package org.austincans.lib.io.sensors.encoders;

import static edu.wpi.first.units.Units.Rotations;

import org.austincans.lib.io.sensors.encoders.AbsoluteEncoderIO.AbsoluteEncoderIOConfigRequests;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CANcoderIO implements AbsoluteEncoderIO<CANcoderIO.CANcoderIOConfigRequests, CANcoderConfig> {
    private final CANcoder encoder;

    private final CANcoderConfiguration vendorConfig;

    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    protected CANcoderIO(int deviceId, CANBus canbus) {
        encoder = new CANcoder(deviceId, canbus);

        vendorConfig = new CANcoderConfiguration();

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

    @Override
    public void applyConfigRequests(CANcoderIOConfigRequests configRequests) {
        configRequests.magnetSensorConfigChanged = false;

        vendorConfig.MagnetSensor.MagnetOffset = configRequests.config.magnetOffset.in(Rotations);
        vendorConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 
            configRequests.config.absoluteSensorDiscontinuityPoint.in(Rotations);
        vendorConfig.MagnetSensor.SensorDirection = configRequests.config.sensorDirectionInverted 
            ? SensorDirectionValue.CounterClockwise_Positive 
            : SensorDirectionValue.Clockwise_Positive;

        encoder.getConfigurator().apply(vendorConfig.MagnetSensor);
    }

    public class CANcoderIOConfigRequests extends AbsoluteEncoderIOConfigRequests<CANcoderConfig> {
        protected CANcoderConfig config;
        protected boolean magnetSensorConfigChanged;

        public void magnetOffset(Angle offset) {
            magnetSensorConfigChanged |= !config.magnetOffset.isEquivalent(offset);
            config.magnetOffset = offset;
        }
        public void absoluteSensorDiscontinuityPoint(Angle discontinuityPoint) {
            magnetSensorConfigChanged |= 
                !config.absoluteSensorDiscontinuityPoint.isEquivalent(discontinuityPoint);
            config.absoluteSensorDiscontinuityPoint = discontinuityPoint;
        }
        public void sensorDirectionInverted(boolean inverted) {
            magnetSensorConfigChanged |= config.sensorDirectionInverted != inverted;
            config.sensorDirectionInverted = inverted;
        }
    }
}
