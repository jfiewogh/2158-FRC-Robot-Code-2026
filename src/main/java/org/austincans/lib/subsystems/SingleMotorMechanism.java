package org.austincans.lib.subsystems;

import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.motors.MotorIO;
import org.austincans.lib.io.motors.configs.MotorConfig;
import org.austincans.lib.io.motors.configs.requests.MotorIOConfigRequests;
import org.austincans.lib.io.motors.inputs.MotorIOInputs;
import org.austincans.lib.io.motors.outputs.MotorIOOutputs;
import org.austincans.lib.util.FullSubsystem;

public class SingleMotorMechanism extends FullSubsystem {
    private final MotorIO<InputsType, > io;
    private final MotorIOInputs inputs;
    private final MotorIOOutputs<?,?,?> outputs;
    private final MotorIOConfigRequests<? extends ConfigRequests> configRequests = new MotorIOConfigRequests<>();

    public SingleMotorMechanism(MotorIO<?,?,?,?,?,?> io, MotorConfig config) {
        this.io = io;
        configRequests.setConfig(config);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.applyConfigRequests(configRequests);
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }
}
