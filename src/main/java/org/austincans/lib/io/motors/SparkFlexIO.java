package org.austincans.lib.io.motors;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

public class SparkFlexIO extends SparkIO<SparkFlex, SparkFlexConfig> {
    SparkFlexIO(SparkConfig config) {
        super(new SparkFlex(config.deviceId, config.type), new SparkFlexConfig(), config);
    }
}