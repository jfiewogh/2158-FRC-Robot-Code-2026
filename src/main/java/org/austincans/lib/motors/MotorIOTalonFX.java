package org.austincans.lib.motors;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class MotorIOTalonFX extends MotorIOReal {
    private final TalonFX motor;

    private final MotionMagicVelocityVoltage velocityVoltage = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0);

    protected MotorIOTalonFX(int deviceId, CANBus canbus, TalonFXConfiguration motorConfig) {
        super(deviceId);
        motor = new TalonFX(deviceId, canbus);
        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateRecords(MotorIORecords records) {
        records.connected = motor.isConnected();
        records.positionRad = motor.getPosition().getValue().in(Radians);
        records.velocityRadPerSec = motor.getVelocity().getValue().in(RadiansPerSecond);
        records.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        records.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        records.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        records.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
        records.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyRequests(MotorIORequests requests) {
        switch (requests.requestMode) {
            case VOLTAGE -> setVoltage(requests.voltage);
            case VELOCITY -> setVelocity(requests.velocityRadPerSec);
            case POSITION -> setPosition(requests.positionRad);
        }
    }

    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    private void setVelocity(double velocityRadPerSec) {
        motor.setControl(velocityVoltage.withVelocity(
            AngularVelocity.ofBaseUnits(velocityRadPerSec, RadiansPerSecond))); 
    }

    private void setPosition(double positionRad) {
        motor.setControl(positionVoltage.withPosition(
            Units.radiansToRotations(positionRad)));
    }

    public void setEncoderPosition(double positionRotations) {
        motor.setPosition(positionRotations);
    }
}
