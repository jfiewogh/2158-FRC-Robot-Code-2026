package org.austincans.lib.io.motors;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class SparkFlexIO extends SparkIO {
    protected SparkFlexIO(int deviceId, MotorType motorType) {
        super(new SparkFlex(deviceId, motorType), new SparkFlexConfig());
    }
}