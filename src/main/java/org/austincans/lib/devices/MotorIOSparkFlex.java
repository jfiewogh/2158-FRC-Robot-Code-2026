package org.austincans.lib.devices;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

/** You can extend or update this class to add config */
public class MotorIOSparkFlex extends MotorIOBase {
    protected final SparkFlex motor;
    protected final RelativeEncoder encoder;
    protected final SparkClosedLoopController controller;

    public MotorIOSparkFlex(int deviceId, MotorType type, SparkFlexConfig config, ResetMode resetMode, PersistMode persistMode) {
        super(deviceId);
        motor = new SparkFlex(deviceId, type);
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        config.encoder.positionConversionFactor(Math.PI * 2);
        motor.configure(config, resetMode, persistMode);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRad = encoder.getPosition();
        inputs.velocityRadPerSec = encoder.getVelocity();
        inputs.voltage = motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double voltage) {
        controller.setSetpoint(voltage, ControlType.kVoltage);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        controller.setSetpoint(velocityRadPerSec, ControlType.kVoltage);
    }

    @Override
    public void setPosition(double positionRad) {
        controller.setSetpoint(positionRad, ControlType.kVoltage);
    }
}
