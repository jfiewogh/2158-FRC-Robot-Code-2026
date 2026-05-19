package org.austincans.lib.motors;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

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
    public void updateInputs(MotorIOInputs inputs) {
        inputs.connected = motor.isConnected();
        inputs.position = motor.getPosition().getValue();
        inputs.velocity = motor.getVelocity().getValue();
        inputs.voltage = motor.getMotorVoltage().getValue();
        inputs.supplyCurrent = motor.getSupplyCurrent().getValue();
        inputs.statorCurrent = motor.getStatorCurrent().getValue();
        inputs.torqueCurrent = motor.getTorqueCurrent().getValue();
        inputs.temperature = motor.getDeviceTemp().getValue();
    }

    @Override
    public void applyOutputs(MotorIOOutputs outputs) {
        switch (outputs.mode) {
            case VOLTAGE -> setVoltage(outputs.voltage);
            case VELOCITY -> setVelocity(outputs.velocity);
            case POSITION -> setPosition(outputs.position);
        }
    }

    private void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.baseUnitMagnitude());
    }

    public TalonFX getMotor(){
        return motor;
    }

    private void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityVoltage.withVelocity(velocity)); 
    }

    private void setPosition(Angle position) {
        motor.setControl(positionVoltage.withPosition(position));
    }

    public void setEncoderPosition(double positionRotations) {
        motor.setPosition(positionRotations);
    }
}
