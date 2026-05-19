package org.austincans.lib.hardware.motors;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class TalonFXIO extends TalonIO {
    private final TalonFX motor;

    private final MotionMagicVelocityVoltage velocityVoltage = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0);

    protected TalonFXIO(int deviceId, CANBus canbus, TalonFXConfiguration motorConfig) {
        super(deviceId);
        motor = new TalonFX(deviceId, canbus);
        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(TalonIOInputs inputs) {
        inputs.setConnected(motor.isConnected());
        inputs.setPosition(motor.getPosition().getValue());
        inputs.setVelocity(motor.getVelocity().getValue());
        inputs.setVoltage(motor.getMotorVoltage().getValue());
        inputs.setSupplyCurrent(motor.getSupplyCurrent().getValue());
        inputs.setStatorCurrent(motor.getStatorCurrent().getValue());
        inputs.setTorqueCurrent(motor.getTorqueCurrent().getValue());
        inputs.setTemperature(motor.getDeviceTemp().getValue());
    }

    @Override
    public void applyOutputs(MotorIOOutputs outputs) {
        switch (outputs.getMode()) {
            case VOLTAGE -> setVoltage(outputs.getVoltage());
            case VELOCITY -> setVelocity(outputs.getVelocity());
            case POSITION -> setPosition(outputs.getPosition());
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

    @Override
    public void registerSelfCheckHardware(AdvancedSubsystem subsystem, String label) {
        subsystem.registerHardware(label, motor);
    }
}
