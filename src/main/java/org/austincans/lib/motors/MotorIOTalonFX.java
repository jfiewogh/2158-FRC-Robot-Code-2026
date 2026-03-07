package org.austincans.lib.motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// TODO: Complete this class
/** You can extend or update this class to add config */
public class MotorIOTalonFX extends MotorIOReal {
    @SuppressWarnings("unused")
    private final TalonFX motor;

    protected MotorIOTalonFX(int deviceId, String canbus, PIDFFConfig pidffConfig, TalonFXConfiguration motorConfig) {
        super(deviceId);
        motor = new TalonFX(deviceId, new CANBus(canbus));
        motorConfig.Slot0.kP = pidffConfig.kP();
        motorConfig.Slot0.kI = pidffConfig.kI();
        motorConfig.Slot0.kD = pidffConfig.kD();
        motorConfig.Slot0.kS = pidffConfig.kS();
        motorConfig.Slot0.kG = pidffConfig.kG();
        motorConfig.Slot0.kV = pidffConfig.kV();
        motorConfig.Slot0.kA = pidffConfig.kA();
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void setVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocity'");
    }

    @Override
    public void setPosition(double positionRad) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }
}
