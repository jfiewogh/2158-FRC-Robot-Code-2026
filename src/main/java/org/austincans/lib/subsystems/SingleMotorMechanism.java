package org.austincans.lib.subsystems;

import org.austincans.lib.io.motors.MotorIO;
import org.austincans.lib.io.motors.inputs.MotorIOInputs;
import org.austincans.lib.io.motors.outputs.MotorIOOutputs;
import org.austincans.lib.util.FullSubsystem;

/** Without config requests */
// TODO: Maybe config requests should be part of outputs?
public class SingleMotorMechanism extends FullSubsystem {
    private final MotorIO io;
    private final MotorIOInputs inputs;
    private final MotorIOOutputs outputs = new MotorIOOutputs();

    public SingleMotorMechanism(MotorIO io, MotorIOInputs inputs) {
        this.io = io;
        this.inputs = inputs;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }
}
