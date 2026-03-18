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
