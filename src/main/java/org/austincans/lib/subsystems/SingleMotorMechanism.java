package org.austincans.lib.subsystems;

import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.motors.MotorIO;
import org.austincans.lib.io.motors.inputs.MotorIOInputs;
import org.austincans.lib.io.motors.outputs.MotorIOOutputs;
import org.austincans.lib.util.FullSubsystem;

public class SingleMotorMechanism extends FullSubsystem {
    protected final MotorIO io;
    protected final MotorIOInputs inputs;
    protected final MotorIOOutputs outputs = new MotorIOOutputs();
    protected final ConfigRequests configRequests; 

    public SingleMotorMechanism(MotorIO io, MotorIOInputs inputs, ConfigRequests configRequests) {
        this.io = io;
        this.inputs = inputs;
        this.configRequests = configRequests;
        applyConfigRequests();
    }

    @Override
    public void periodic() {
        updateInputs();
    }

    @Override
    public void periodicAfterScheduler() {
        applyOutputs();
    }

    public final void updateInputs() {
        io.updateInputs(inputs);
    }

    public final void applyOutputs() {
        io.applyOutputs(outputs);
    }

    public final void applyConfigRequests() {
        io.applyConfigRequests(configRequests);
    }
}
