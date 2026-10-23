package org.austincans.lib.subsystems.hood;

import org.austincans.lib.configs.AbsoluteEncoderConfig;
import org.austincans.lib.configs.MotorConfigSpark;
import org.austincans.lib.configs.MotorConfigTalon;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.revrobotics.spark.config.SparkFlexConfig;

public class HoodConfig{
    public SparkConfig motorConfig;
    public SparkFlexConfig sparkFlexConfig;
    public AbsoluteEncoderConfig encoderConfig;
    public TalonConfig motorConfigTalon;
    public TalonFXConfiguration talonConfig;
    public TalonFXSConfiguration talonFXSConfig;
    public int ServoLinearId;
    public double rotationalInertiaMetersSq;
    public double gearing;
}

