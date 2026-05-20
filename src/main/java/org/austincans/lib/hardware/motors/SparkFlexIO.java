package org.austincans.lib.hardware.motors;

import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

/** You can extend or update this class to add config */
public class SparkFlexIO extends SparkIO<SparkFlex, SparkFlexConfig> {
    SparkFlexIO(int deviceId, MotorType motorType, SparkFlexConfig config) {
        super(new SparkFlex(deviceId, motorType), config);
    }

    @Override
    public void registerSelfCheckHardware(AdvancedSubsystem subsystem, String label) {
        subsystem.registerHardware(label, motor);
    }
}
