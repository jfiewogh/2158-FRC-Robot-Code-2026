package org.austincans.frc2158.subsystems.vision;

import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightResults;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.RawDetection;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ObjectDetection {
    public enum GamePieceType {
        // 2025
        CORAL("Coral", 0.1),
        ALGAE("Algae", 0.1),
        // 2026
        // TODO: name should be same as in detector model
        FUEL("Fuels", Units.inchesToMeters(5.91 * 0.5));

        private final String className;
        private final double radiusMeters;

        private GamePieceType(String className, double radiusMeters) {
            this.className = className;
        }

        public String getClassName() {
            return className;
        }

        public double getRadiusMeters() {
            return radiusMeters;
        }
    }

    private final ObjectDetectionIO io;
    private final ObjectDetectionIOInputsAutoLogged inputs;

    public ObjectDetection(ObjectDetectionIO io) {
        this.io = io;
    }

    public Pose3d getNearestPose(){
        return inputs.nearestPose;
    }

    public Pose3d[] getGamePiecePoses() {
        return inputs.gamePiecePoses;
    }
}
