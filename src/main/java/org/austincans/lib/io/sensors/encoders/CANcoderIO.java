package org.austincans.lib.io.sensors.encoders;

import static edu.wpi.first.units.Units.Rotations;

import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.Inputs;
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

public class CANcoderIO implements AbsoluteEncoderIO {
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
    public void updateInputs(Inputs inputs) {
        CANcoderIOInputs canCoderInputs = (CANcoderIOInputs) inputs;

        StatusSignal.refreshAll(
            absolutePosition, position, velocity, 
            hardwareFault, bootDuringEnable, badMagnet, undervoltage,
            stickyFaultHardware, stickyFaultBootDuringEnable, stickyFaultBadMagnet, stickyFaultUndervoltage);

        canCoderInputs.connected = encoder.isConnected();
        canCoderInputs.absolutePosition = absolutePosition.getValue();
        canCoderInputs.position = position.getValue();
        canCoderInputs.velocity = velocity.getValue();

        canCoderInputs.faultHardware = hardwareFault.getValue();
        canCoderInputs.faultBootDuringEnable = bootDuringEnable.getValue();
        canCoderInputs.faultBadMagnet = badMagnet.getValue();
        canCoderInputs.faultUndervoltage = undervoltage.getValue();

        canCoderInputs.stickyFaultHardware = stickyFaultHardware.getValue();
        canCoderInputs.stickyFaultBootDuringEnable = stickyFaultBootDuringEnable.getValue();
        canCoderInputs.stickyFaultBadMagnet = stickyFaultBadMagnet.getValue();
        canCoderInputs.stickyFaultUndervoltage = stickyFaultUndervoltage.getValue();
    }

    @Override
    public void applyConfigRequests(ConfigRequests configRequests) {
        CANcoderIOConfigRequests canCoderConfigRequests = (CANcoderIOConfigRequests) configRequests;
        CANcoderConfig canCoderConfig = (CANcoderConfig) canCoderConfigRequests.getConfig();

        vendorConfig.MagnetSensor.MagnetOffset = canCoderConfig.magnetOffset.in(Rotations);
        vendorConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 
            canCoderConfig.absoluteSensorDiscontinuityPoint.in(Rotations);
        vendorConfig.MagnetSensor.SensorDirection = canCoderConfig.sensorDirectionInverted 
            ? SensorDirectionValue.CounterClockwise_Positive 
            : SensorDirectionValue.Clockwise_Positive;

        encoder.getConfigurator().apply(vendorConfig.MagnetSensor);
    }

    @Override
    public void clearStickyFaults() {
        encoder.clearStickyFaults();
    }
}
