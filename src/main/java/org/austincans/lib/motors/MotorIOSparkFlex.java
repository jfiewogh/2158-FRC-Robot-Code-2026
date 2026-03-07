package org.austincans.lib.motors;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

/** You can extend or update this class to add config */
public class MotorIOSparkFlex extends MotorIOSpark<SparkFlex, SparkFlexConfig> {
    protected MotorIOSparkFlex(int deviceId, MotorType motorType, PIDFFConfig pidffConfig, SoftLimitConfig softLimitConfig, IdleMode idleMode, SparkFlexConfig config) {
        super(new SparkFlex(deviceId, motorType), pidffConfig, softLimitConfig, idleMode, config);
    }
}
