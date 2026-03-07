package org.austincans.lib.motors;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/** You can extend or update this class to add config */
public class MotorIOSparkMax extends MotorIOSpark<SparkMax, SparkMaxConfig> {
    protected MotorIOSparkMax(int deviceId, MotorType motorType, SparkMaxConfig config) {
        super(new SparkMax(deviceId, motorType), config);
    }
}
