package org.austincans.lib.motors;

import org.austincans.lib.motors.MotorIO.MotorIOInputs;

public class Motor {
    private final MotorIO io;
    private final MotorIOInputs inputs;

    public Motor(MotorIO io, MotorIOInputs inputs) {

    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    
}
