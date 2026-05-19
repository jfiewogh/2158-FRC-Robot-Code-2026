package org.austincans.lib.hardware.motors;

import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

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

    protected TalonFXSIO(int deviceId, CANBus canbus, TalonFXSConfiguration motorConfig) {
        super(deviceId);
        motor = new TalonFXS(deviceId, canbus);
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

    @Override
    public void registerSelfCheckHardware(AdvancedSubsystem subsystem, String label) {
        subsystem.registerHardware(label, motor);
    }
}
