package org.austincans.lib.motors;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.util.Units;

public abstract class MotorIOSpark<Motor extends SparkBase, Config extends SparkBaseConfig> extends MotorIOReal {
    protected final Motor motor;
    protected final RelativeEncoder encoder;
    protected final SparkClosedLoopController controller;

    protected MotorIOSpark(Motor motor, Config config) {
        super(motor.getDeviceId());
        this.motor = motor;
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
        motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRad = Units.rotationsToRadians(encoder.getPosition());
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        inputs.appliedVolts = motor.getBusVoltage();
        inputs.statorCurrentAmps = motor.getOutputCurrent();
        inputs.temperatureCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double voltage) {
        controller.setSetpoint(voltage, ControlType.kVoltage);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        controller.setSetpoint(velocityRadPerSec, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void setPosition(double positionRad) {
        controller.setSetpoint(positionRad, ControlType.kMAXMotionVelocityControl);
    }

    public void setEncoderPosition(double positionRad) {
        encoder.setPosition(positionRad);
    }
}
