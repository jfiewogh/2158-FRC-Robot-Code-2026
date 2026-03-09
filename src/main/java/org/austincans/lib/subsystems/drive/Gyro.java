package org.austincans.lib.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation3d;

/** Wrapper class for GyroIO */
public class Gyro {
    private final GyroIO io;
    private final GyroIOInputs inputs;

    public Gyro(GyroIO io, GyroIOInputs inputs) {
        this.io = io;
        this.inputs = inputs;
    }

    public void periodic() {
        
    }

    public Rotation3d getRotation3d() {


    }
}
