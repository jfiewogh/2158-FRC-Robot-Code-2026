package org.austincans.lib.motors;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class MotorIOTalonFX extends MotorIOReal {
    private final TalonFX motor;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    protected MotorIOTalonFX(int deviceId, CANBus canbus, TalonFXConfiguration motorConfig) {
        super(deviceId);
        motor = new TalonFX(deviceId, canbus);
        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRad = getPositionRad();
        inputs.velocityRadPerSec = getVelocityRadPerSec(); 
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
        inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
    }

    public double getPositionRad() {
        return motor.getPosition().getValue().in(Radians);
    }

    public double getVelocityRadPerSec() {
        return motor.getVelocity().getValue().in(RadiansPerSecond);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        motor.setControl(velocityVoltage.withVelocity(AngularVelocity.ofBaseUnits(velocityRadPerSec, RadiansPerSecond))); 
    }

    @Override
    public void setPosition(double positionRad) {
        motor.setControl(positionVoltage.withPosition(positionRad));
    }

    public TalonFX getMotor() {
        return motor;
    }
}
