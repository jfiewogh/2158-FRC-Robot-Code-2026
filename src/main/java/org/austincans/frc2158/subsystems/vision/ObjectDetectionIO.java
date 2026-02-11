package org.austincans.frc2158.subsystems.vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface ObjectDetectionIO {
    @AutoLog
    public class ObjectDetectionIOInputs {
        public Pose3d nearestPose = new Pose3d();
        public Pose3d[] gamePiecePoses = {};
    }

    public void updateInputs(ObjectDetectionIOInputs inputs);
}
