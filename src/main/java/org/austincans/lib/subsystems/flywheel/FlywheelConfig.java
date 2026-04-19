package org.austincans.lib.subsystems.flywheel;

import org.austincans.lib.configs.MotorConfigSpark;
import org.austincans.lib.configs.MotorConfigTalon;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.revrobotics.spark.config.SparkFlexConfig;

public class FlywheelConfig {
    public MotorConfigSpark motorSparkConfig;
    public MotorConfigTalon motorTalonConfig;
    public SparkFlexConfig sparkFlexConfig;
    public TalonFXConfiguration talonFXConfig;
    public TalonFXSConfiguration talonFXSConfig;
}