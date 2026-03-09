package org.austincans.lib.motors;

import org.austincans.lib.motors.MotorIO.MotorIOInputs;

/** 
 * Wrapper class for interacting with MotorIO. 
 * Should be instance variables of subsystems and IOs. 
 */
public class Motor {
    private final MotorIO io;
    private final MotorIOInputs inputs;

    protected Motor(MotorIO io, MotorIOInputs inputs) {
        this.io = io;
        this.inputs = inputs;
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void setVelocity(double velocityRadPerSec) {
        io.setVelocity(velocityRadPerSec);
    }

    public void setPosition(double positionRad) {
        io.setPosition(positionRad);
    }
}
