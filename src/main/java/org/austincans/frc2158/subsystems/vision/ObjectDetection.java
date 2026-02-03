package org.austincans.frc2158.subsystems.vision;

import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightResults;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ObjectDetection {
    private final String limelightName;
    private final String className;

    public ObjectDetection(String limelightName, String className) {
        this.limelightName = limelightName;
        this.className = className;
    }

    public Pose3d[] calculateGamePiecePoses() {
        LimelightResults latestResults = LimelightHelpers.getLatestResults(limelightName);
        LimelightTarget_Detector[] detectorResults = latestResults.targets_Detector;

        Pose3d robotPose = LimelightHelpers.getBotPose3d(limelightName);

        LimelightTarget_Detector closestResult = detectorResults[0];

        for (LimelightTarget_Detector result : detectorResults) {
            if (result.className == className) {
                
                // Use these values to calculate Pose3d
                Rotation2d yaw = Rotation2d.fromDegrees(result.tx);
                Rotation2d pitch = Rotation2d.fromDegrees(result.ty);

                double size = result.ta;

                System.out.println(yaw.getDegrees() + " " + pitch.getDegrees() + " " + size);
            }
        }

        return new Pose3d[] {};
    }
}
