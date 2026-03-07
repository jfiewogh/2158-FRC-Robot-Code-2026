package org.austincans.lib.motors;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

/** You can extend or update this class to add config */
public class MotorIOSparkFlex extends MotorIOSpark<SparkFlex, SparkFlexConfig> {
    protected MotorIOSparkFlex(int deviceId, MotorType motorType, SparkFlexConfig config) {
        super(new SparkFlex(deviceId, motorType), config);
    }
}
