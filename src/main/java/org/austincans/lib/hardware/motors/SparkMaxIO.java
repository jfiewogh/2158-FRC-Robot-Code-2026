package org.austincans.lib.hardware.motors;

import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/** You can extend or update this class to add config */
public class SparkMaxIO extends SparkIO<SparkMax, SparkMaxConfig> {
    protected SparkMaxIO(int deviceId, MotorType motorType, SparkMaxConfig config) {
        super(new SparkMax(deviceId, motorType), config);
    }

    @Override
    public void registerSelfCheckHardware(AdvancedSubsystem subsystem, String label) {
        subsystem.registerHardware(label, motor);
    }
}
