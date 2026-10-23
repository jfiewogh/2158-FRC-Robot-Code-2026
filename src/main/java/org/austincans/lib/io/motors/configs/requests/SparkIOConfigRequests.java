package org.austincans.lib.io.motors.configs.requests;

import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.motors.configs.NeutralMode;
import org.austincans.lib.io.motors.configs.SoftLimitConfig;
import org.austincans.lib.util.math.physical.Gearbox;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import org.austincans.lib.io.motors.configs.SparkConfig;
import org.austincans.lib.io.motors.configs.SparkConfig.*;

public class SparkIOConfigRequests extends ConfigRequests {
    public void softLimitConfig(SoftLimitConfig softLimitConfig) {
        SparkConfig sparkConfig = (SparkConfig) config;
        configChanged |= !sparkConfig.softLimitConfig.equals(softLimitConfig);
        sparkConfig.softLimitConfig = softLimitConfig;
    }
    public void neutralMode(NeutralMode neutralMode) {
        SparkConfig sparkConfig = (SparkConfig) config;
        configChanged |= sparkConfig.neutralMode != neutralMode;
        sparkConfig.neutralMode = neutralMode;
    }
    public void inverted(boolean inverted) {
        SparkConfig sparkConfig = (SparkConfig) config;
        configChanged |= sparkConfig.inverted != inverted;
        sparkConfig.inverted = inverted;
    }
    public void smartCurrentStallLimit(Current smartCurrentStallLimit) {
        SparkConfig sparkConfig = (SparkConfig) config;
        configChanged |= sparkConfig.smartCurrentStallLimit != smartCurrentStallLimit;
        sparkConfig.smartCurrentStallLimit = smartCurrentStallLimit;
    }
    public void voltageCompensation(Voltage voltage) {
        SparkConfig sparkConfig = (SparkConfig) config;
        configChanged |= !sparkConfig.voltageCompensation.isEquivalent(voltage);
        sparkConfig.voltageCompensation = voltage;
    }
    public void sensorToMechanismGearbox(Gearbox gearbox) {
        SparkConfig sparkConfig = (SparkConfig) config;
        configChanged |= !sparkConfig.sensorToMechanismGearbox.equals(gearbox);
        sparkConfig.sensorToMechanismGearbox = gearbox;
    }
    public void encoderConfig(EncoderConfig encoderConfig) {
        SparkConfig sparkConfig = (SparkConfig) config;
        configChanged |= !sparkConfig.encoderConfig.equals(encoderConfig);
        sparkConfig.encoderConfig = encoderConfig;
    }
    public void absoluteEncoderConfig(AbsoluteEncoderConfig absoluteEncoderConfig) {
        SparkConfig sparkConfig = (SparkConfig) config;
        configChanged |= !sparkConfig.absoluteEncoderConfig.equals(absoluteEncoderConfig);
        sparkConfig.absoluteEncoderConfig = absoluteEncoderConfig;
    }
    public void closedLoopConfig(ClosedLoopConfig closedLoopConfig) {
        SparkConfig sparkConfig = (SparkConfig) config;
        configChanged |= !sparkConfig.closedLoopConfig.equals(closedLoopConfig);
        sparkConfig.closedLoopConfig = closedLoopConfig;
    }
    public void followerConfig(FollowerConfig followerConfig) {
        SparkConfig sparkConfig = (SparkConfig) config;
        configChanged |= !sparkConfig.followerConfig.equals(followerConfig);
        sparkConfig.followerConfig = followerConfig;
    }
    public void signalsConfig(SignalsConfig signalsConfig) {
        SparkConfig sparkConfig = (SparkConfig) config;
        configChanged |= !sparkConfig.signalsConfig.equals(signalsConfig);
        sparkConfig.signalsConfig = signalsConfig;
    }
}