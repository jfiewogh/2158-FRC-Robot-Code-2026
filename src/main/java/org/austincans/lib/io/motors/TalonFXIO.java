package org.austincans.lib.io.motors;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class TalonFXIO extends TalonIO {
    private final TalonFX motor;

    /** 
     * TODO: Config should decide which to used 
     * TODO: Handle PID values for multiple controls (ex. config should include slot such as Slot0, Slot1, etc.)
     */
    private final MotionMagicVelocityVoltage velocityVoltage = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0);

    TalonFXIO(int deviceId, CANBus canbus, TalonFXConfiguration motorConfig) {
        motor = new TalonFX(deviceId, canbus);
        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(TalonIOInputs inputs) {
        inputs.setPosition(motor.getPosition().getValue());
        inputs.setVelocity(motor.getVelocity().getValue());
        inputs.setVoltage(motor.getMotorVoltage().getValue());
        inputs.setSupplyCurrent(motor.getSupplyCurrent().getValue());
        inputs.setStatorCurrent(motor.getStatorCurrent().getValue());
        inputs.setTorqueCurrent(motor.getTorqueCurrent().getValue());
        inputs.setTemperature(motor.getDeviceTemp().getValue());
    }

    @Override
    void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.baseUnitMagnitude());
    }

    @Override
    void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityVoltage.withVelocity(velocity)); 
    }

    @Override
    void setPosition(Angle position) {
        motor.setControl(positionVoltage.withPosition(position));
    }
}
