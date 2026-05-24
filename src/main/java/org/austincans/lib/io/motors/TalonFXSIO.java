package org.austincans.lib.io.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXSIO extends TalonIO {
    private final TalonFXS motor;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private PositionVoltage positionVoltage = new PositionVoltage(0);

    TalonFXSIO(int deviceId, CANBus canbus, TalonFXSConfiguration motorConfig) {
        motor = new TalonFXS(deviceId, canbus);
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

    /** TODO: Remove this method and make setter encoder position part of updateConfig */
    public void setEncoderPosition(double positionRotations) {
        motor.setPosition(positionRotations);
    }
}
