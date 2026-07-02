package org.austincans.lib.io.sensors.encoders;

import static edu.wpi.first.units.Units.Rotations;

import org.austincans.lib.io.sensors.encoders.configs.CANcoderConfig;
import org.austincans.lib.io.sensors.encoders.configs.requests.CANcoderIOConfigRequests;
import org.austincans.lib.io.sensors.encoders.inputs.CANcoderIOInputs;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CANcoderIO implements AbsoluteEncoderIO<CANcoderIOInputs, CANcoderIOConfigRequests, CANcoderConfig> {
    private final CANcoder encoder;

    private final CANcoderConfiguration vendorConfig;

    private final StatusSignal<Angle> absolutePosition;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    private final StatusSignal<Boolean> hardwareFault;
    private final StatusSignal<Boolean> bootDuringEnable;
    private final StatusSignal<Boolean> badMagnet;
    private final StatusSignal<Boolean> undervoltage;

    private final StatusSignal<Boolean> stickyFaultHardware;
    private final StatusSignal<Boolean> stickyFaultBootDuringEnable;
    private final StatusSignal<Boolean> stickyFaultBadMagnet;
    private final StatusSignal<Boolean> stickyFaultUndervoltage;

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

        stickyFaultHardware = encoder.getStickyFault_Hardware();
        stickyFaultBootDuringEnable = encoder.getStickyFault_BootDuringEnable();
        stickyFaultBadMagnet = encoder.getStickyFault_BadMagnet();
        stickyFaultUndervoltage = encoder.getStickyFault_Undervoltage();
    }

    @Override
    public void updateInputs(CANcoderIOInputs inputs) {
        StatusSignal.refreshAll(
            absolutePosition, position, velocity, 
            hardwareFault, bootDuringEnable, badMagnet, undervoltage,
            stickyFaultHardware, stickyFaultBootDuringEnable, stickyFaultBadMagnet, stickyFaultUndervoltage);

        inputs.connected = encoder.isConnected();
        inputs.absolutePosition = absolutePosition.getValue();
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();

        inputs.faultHardware = hardwareFault.getValue();
        inputs.faultBootDuringEnable = bootDuringEnable.getValue();
        inputs.faultBadMagnet = badMagnet.getValue();
        inputs.faultUndervoltage = undervoltage.getValue();

        inputs.stickyFaultHardware = stickyFaultHardware.getValue();
        inputs.stickyFaultBootDuringEnable = stickyFaultBootDuringEnable.getValue();
        inputs.stickyFaultBadMagnet = stickyFaultBadMagnet.getValue();
        inputs.stickyFaultUndervoltage = stickyFaultUndervoltage.getValue();
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

    @Override
    public void clearStickyFaults() {
        encoder.clearStickyFaults();
    }
}
