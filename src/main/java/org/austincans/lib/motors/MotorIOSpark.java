package org.austincans.lib.motors;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public abstract class MotorIOSpark<Motor extends SparkBase, MotorConfig extends SparkBaseConfig> extends MotorIOReal {
    protected final Motor motor;
    protected final RelativeEncoder encoder;
    protected final SparkClosedLoopController controller;

    protected MotorIOSpark(Motor motor, PIDFFConfig pidffConfig, SoftLimitConfig softLimitConfig, IdleMode idleMode, MotorConfig motorConfig) {
        super(motor.getDeviceId());
        this.motor = motor;
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        motorConfig.closedLoop.pid(pidffConfig.kP(), pidffConfig.kI(), pidffConfig.kD());
        motorConfig.closedLoop.feedForward.kS(pidffConfig.kS());
        motorConfig.closedLoop.feedForward.kG(pidffConfig.kG());
        motorConfig.closedLoop.feedForward.kV(pidffConfig.kV());
        motorConfig.closedLoop.feedForward.kA(pidffConfig.kA());
        motorConfig.softLimit.forwardSoftLimitEnabled(softLimitConfig.forwardSoftLimitEnabled());
        motorConfig.softLimit.forwardSoftLimit(softLimitConfig.forwardSoftLimit());
        motorConfig.softLimit.reverseSoftLimitEnabled(softLimitConfig.reverseSoftLimitEnabled());
        motorConfig.softLimit.reverseSoftLimit(softLimitConfig.reverseSoftLimit());
        motorConfig.idleMode(idleMode);

        motor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRad = encoder.getPosition();
        inputs.velocityRadPerSec = encoder.getVelocity();
        inputs.appliedVolts = motor.getBusVoltage();
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

    public Motor getMotor() {
        return motor;
    }

    public RelativeEncoder getEncder(){
        return motor.getEncoder();
    }

    public void setEncoderPosition(double positionRad) {
        encoder.setPosition(positionRad);
    }
}
