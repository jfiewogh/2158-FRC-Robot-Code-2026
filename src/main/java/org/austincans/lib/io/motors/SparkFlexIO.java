package org.austincans.lib.io.motors;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

/** You can extend or update this class to add config */
public class SparkFlexIO extends SparkIO<SparkFlex, SparkFlexConfig> {
    /** TODO: Update this constructor to only take in SparkConfig */
    SparkFlexIO(int deviceId, MotorType motorType, SparkFlexConfig config) {
        super(new SparkFlex(deviceId, motorType), config);
    }

}
