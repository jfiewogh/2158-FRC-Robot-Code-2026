package org.austincans.lib.io.motors;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/** You can extend or update this class to add config */
public class SparkMaxIO extends SparkIO<SparkMax, SparkMaxConfig> {
    /** TODO: Update this constructor to only take in SparkConfig */
    SparkMaxIO(int deviceId, MotorType motorType, SparkMaxConfig config) {
        super(new SparkMax(deviceId, motorType), config);
    }
}
