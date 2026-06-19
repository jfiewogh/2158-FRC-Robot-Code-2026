package org.austincans.lib.io.sensors.encoders;

import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;

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

    private final StatusSignal<Boolean> hardwareFault;
    private final StatusSignal<Boolean> bootDuringEnabled;
    private final StatusSignal<Boolean> badMagnet;
    private final StatusSignal<Boolean> undervoltage;

    private final ArrayList<String> faults = new ArrayList<>();

    protected CANcoderIO(int deviceId, CANBus canbus) {
        encoder = new CANcoder(deviceId, canbus);

        vendorConfig = new CANcoderConfiguration();

        absolutePosition = encoder.getAbsolutePosition();
        position = encoder.getPosition(); // relative to start
        velocity = encoder.getVelocity();

        hardwareFault = encoder.getFault_Hardware();
        bootDuringEnabled = encoder.getFault_BootDuringEnable();
        badMagnet = encoder.getFault_BadMagnet();
        undervoltage = encoder.getFault_Undervoltage();
    }

    @Override
    public void updateInputs(AbsoluteEncoderIOInputs inputs) {
        inputs.connected = encoder.isConnected();
        inputs.absolutePosition = absolutePosition.getValue();
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.activeFaults = getActiveFaults();
    }

    // TODO: Move this into new AbstractAbsoluteEncoderIO class and change to protected, similarly to AbstractMotorIO
    private String[] getActiveFaults() {
        faults.clear();
        StatusSignal.refreshAll(hardwareFault, bootDuringEnabled, badMagnet, undervoltage);
        if (hardwareFault.getValue()) {
            faults.add("Hardware fault detected");
        }
        if (bootDuringEnabled.getValue()) {
            faults.add("Device booted while enabled");
        }
        if (badMagnet.getValue()) {
            faults.add("Magnet too weak");
        }
        if (undervoltage.getValue()) {
            faults.add("Under voltage");
        }
        return faults.toArray(new String[0]);
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
