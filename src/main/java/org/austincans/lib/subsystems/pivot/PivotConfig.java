package org.austincans.lib.subsystems.pivot;
import org.austincans.lib.configs.AbsoluteEncoderConfig;
import org.austincans.lib.configs.MotorConfigSpark;
import org.austincans.lib.configs.MotorConfigTalon;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.revrobotics.spark.config.SparkFlexConfig;

public class PivotConfig {
    public MotorConfigSpark motorSparkConfig;
    public MotorConfigTalon motorTalonConfig;
    public SparkFlexConfig sparkFlexConfig;
    public TalonFXSConfiguration talonFXSConfig;
    public AbsoluteEncoderConfig absoluteEncoderConfig;
}