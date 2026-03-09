package org.austincans.lib.subsystems.drive;

import org.austincans.lib.subsystems.drive.GyroIO.GyroIOInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/** 
 * Wrapper class for GyroIO 
 * If needed, add more methods
 */
public class Gyro {
    private final GyroIO io;
    private final GyroIOInputs inputs;

    public Gyro(GyroIO io, GyroIOInputs inputs) {
        this.io = io;
        this.inputs = inputs;
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public Rotation2d getYaw() {
        return inputs.yawPosition;
    }

    public Rotation2d getPitch() {
        return inputs.pitchPosition;
    }

    public Rotation2d getRoll() {
        return inputs.rollPosition;
    }

    public Rotation3d getRotation3d() {
        return new Rotation3d(
            inputs.rollPosition.getRadians(), 
            inputs.pitchPosition.getRadians(), 
            inputs.yawPosition.getRadians());
    }
}
