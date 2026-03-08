package org.austincans.lib.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class MotorIOTalonFXS extends MotorIOReal {
    private final TalonFXS motor;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private PositionVoltage positionVoltage = new PositionVoltage(0);

    protected MotorIOTalonFXS(int deviceId, CANBus canbus, TalonFXSConfiguration motorConfig) {
        super(deviceId);
        motor = new TalonFXS(deviceId, canbus);
        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRad = motor.getPosition().getValue().in(Radians);
        inputs.velocityRadPerSec = motor.getVelocity().getValue().in(RadiansPerSecond);
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
        inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        motor.setControl(velocityVoltage.withVelocity(
            AngularVelocity.ofBaseUnits(velocityRadPerSec, RadiansPerSecond))); 
    }

    @Override
    public void setPosition(double positionRad) {
        motor.setControl(positionVoltage.withPosition(
            Units.radiansToRotations(positionRad)));
    }
}
