package org.austincans.lib.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

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

    private void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityVoltage.withVelocity(velocity)); 
    }

     private void setPosition(Angle position) {
        motor.setControl(positionVoltage.withPosition(position));
    }

    public TalonFXS getMotor(){
        return motor;
    }

    public void setEncoderPosition(double positionRotations) {
        motor.setPosition(positionRotations);
    }
}
