package org.austincans.lib.io.motors.configrequests;

import org.austincans.lib.io.motors.configs.NeutralMode;
import org.austincans.lib.io.motors.configs.SoftLimitConfig;
import org.austincans.lib.util.math.physical.Gearbox;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import org.austincans.lib.io.motors.configs.SparkConfig;
import org.austincans.lib.io.motors.configs.SparkConfig.*;

public class SparkIOConfigRequests extends MotorIOConfigRequests<SparkConfig> {
    public void softLimitConfig(SoftLimitConfig softLimitConfig) {
        configChanged |= !config.softLimitConfig.equals(softLimitConfig);
        config.softLimitConfig = softLimitConfig;
    }
    public void neutralMode(NeutralMode neutralMode) {
        configChanged |= config.neutralMode != neutralMode;
        config.neutralMode = neutralMode;
    }
    public void inverted(boolean inverted) {
        configChanged |= config.inverted != inverted;
        config.inverted = inverted;
    }
    public void smartCurrentStallLimit(Current smartCurrentStallLimit) {
        configChanged |= config.smartCurrentStallLimit != smartCurrentStallLimit;
        config.smartCurrentStallLimit = smartCurrentStallLimit;
    }
    public void voltageCompensation(Voltage voltage) {
        configChanged |= !config.voltageCompensation.isEquivalent(voltage);
        config.voltageCompensation = voltage;
    }
    public void sensorToMechanismGearbox(Gearbox gearbox) {
        configChanged |= !config.sensorToMechanismGearbox.equals(gearbox);
        config.sensorToMechanismGearbox = gearbox;
    }
    public void encoderConfig(EncoderConfig encoderConfig) {
        configChanged |= !config.encoderConfig.equals(encoderConfig);
        config.encoderConfig = encoderConfig;
    }
    public void absoluteEncoderConfig(AbsoluteEncoderConfig absoluteEncoderConfig) {
        configChanged |= !config.absoluteEncoderConfig.equals(absoluteEncoderConfig);
        config.absoluteEncoderConfig = absoluteEncoderConfig;
    }
    public void closedLoopConfig(ClosedLoopConfig closedLoopConfig) {
        configChanged |= !config.closedLoopConfig.equals(closedLoopConfig);
        config.closedLoopConfig = closedLoopConfig;
    }
    public void followerConfig(FollowerConfig followerConfig) {
        configChanged |= !config.followerConfig.equals(followerConfig);
        config.followerConfig = followerConfig;
    }
    public void signalsConfig(SignalsConfig signalsConfig) {
        configChanged |= !config.signalsConfig.equals(signalsConfig);
        config.signalsConfig = signalsConfig;
    }
}