package org.austincans.frc2158.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface ObjectDetectionIO {
    @AutoLog
    public class ObjectDetectionIOInputs {
        public Pose3d[] gamePiecePoses = new Pose3d[] {};
    }

    public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
