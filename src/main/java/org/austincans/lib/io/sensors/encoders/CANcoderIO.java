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

public class CANcoderIO implements AbsoluteEncoderIO<CANcoderIO.CANcoderIOInputs, CANcoderIO.CANcoderIOConfigRequests, CANcoderConfig> {
    private final CANcoder encoder;

    private final CANcoderConfiguration vendorConfig;

    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    private final StatusSignal<Boolean> hardwareFault;
    private final StatusSignal<Boolean> bootDuringEnable;
    private final StatusSignal<Boolean> badMagnet;
    private final StatusSignal<Boolean> undervoltage;

    protected CANcoderIO(int deviceId, CANBus canbus) {
        encoder = new CANcoder(deviceId, canbus);

        vendorConfig = new CANcoderConfiguration();

        absolutePosition = encoder.getAbsolutePosition();
        position = encoder.getPosition(); // relative to start
        velocity = encoder.getVelocity();

        hardwareFault = encoder.getFault_Hardware();
        bootDuringEnable = encoder.getFault_BootDuringEnable();
        badMagnet = encoder.getFault_BadMagnet();
        undervoltage = encoder.getFault_Undervoltage();
    }

    public class CANcoderIOInputs extends AbsoluteEncoderIO.AbsoluteEncoderIOInputs {
        public boolean faultHardware;
        public boolean faultBootDuringEnable;
        public boolean faultBadMagnet;
        public boolean faultUndervoltage;
    }

    @Override
    public void updateInputs(CANcoderIOInputs inputs) {
        StatusSignal.refreshAll(absolutePosition, position, velocity, hardwareFault, bootDuringEnable, badMagnet, undervoltage);

        inputs.connected = encoder.isConnected();
        inputs.absolutePosition = absolutePosition.getValue();
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();

        inputs.faultHardware = hardwareFault.getValue();
        inputs.faultBootDuringEnable = bootDuringEnable.getValue();
        inputs.faultBadMagnet = badMagnet.getValue();
        inputs.faultUndervoltage = undervoltage.getValue();
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
