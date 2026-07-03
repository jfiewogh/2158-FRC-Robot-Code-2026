package org.austincans.lib.io.motors;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxIO extends SparkIO {
    protected SparkMaxIO(int deviceId, MotorType motorType) {
        super(new SparkMax(deviceId, motorType), new SparkMaxConfig());
    }
}
