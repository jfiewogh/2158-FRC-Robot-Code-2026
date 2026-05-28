package org.austincans.lib.io.motors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/** You can extend or update this class to add config */
public class SparkMaxIO extends SparkIO<SparkMax, SparkMaxConfig> {
    SparkMaxIO(SparkConfig config) {
        super(new SparkMax(config.deviceId, config.type), new SparkMaxConfig(), config);
    }
}
