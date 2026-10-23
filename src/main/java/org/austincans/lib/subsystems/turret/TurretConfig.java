package org.austincans.lib.subsystems.turret;

import org.austincans.lib.configs.AbsoluteEncoderConfig;
import org.austincans.lib.configs.MotorConfigSpark;
import org.austincans.lib.configs.MotorConfigTalon;
import com.revrobotics.spark.config.SparkFlexConfig;

public class TurretConfig {
    public AbsoluteEncoderConfig encoderConfig;
    public SparkConfig motorSparkConfig;
    public TalonConfig motorTalonConfig;
    public SparkFlexConfig sparkFlexConfig;
}
