package org.austincans.lib.devices;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;

// TODO: Complete this class
/** You can extend or update this class to add config */
public class MotorIOTalonFX extends MotorIOBase {
    @SuppressWarnings("unused")
    private final TalonFX motor;

    public MotorIOTalonFX(int deviceId, String canbus) {
        super(deviceId);
        motor = new TalonFX(deviceId, new CANBus(canbus));
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
