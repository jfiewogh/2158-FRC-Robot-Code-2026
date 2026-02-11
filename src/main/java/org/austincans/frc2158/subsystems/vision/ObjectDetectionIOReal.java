package org.austincans.frc2158.subsystems.vision;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;

import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightResults;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.RawDetection;
import org.austincans.frc2158.subsystems.vision.ObjectDetection.GamePieceType;
import org.austincans.frc2158.subsystems.vision.VisionConstants.LimelightConfig;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class ObjectDetectionIOReal implements ObjectDetectionIO {
    private final String limelightName;
    private final GamePieceType gamePieceType;
    private final LimelightConfig config;
    private int index = 0;

    public ObjectDetectionIOReal(String limelightName, GamePieceType gamePieceType, LimelightConfig config) {
        this.limelightName = limelightName;
        this.gamePieceType = gamePieceType;
        this.config = config;
    }

    private Pose3d getNearestGamePiece() {
        LimelightResults latestResults = LimelightHelpers.getLatestResults(limelightName);
        LimelightTarget_Detector[] detectorResults = latestResults.targets_Detector;

        Pose3d robotPose = LimelightHelpers.getBotPose3d(limelightName);
        Translation3d cameraOffset = new Translation3d(0, 0, 0);
        ArrayList<Pose3d> gamePiecePose3d = new ArrayList<Pose3d>();

        for (LimelightTarget_Detector result : detectorResults) {
            if (result.className == gamePieceType.getClassName()) {
                // Use these values to calculate Pose3d
                Rotation2d yaw = Rotation2d.fromDegrees(result.tx);
                Rotation2d pitch = Rotation2d.fromDegrees(result.ty);

                double areaPixels = result.ta * config.getWidthPixels() * config.getHeightPixels();

                // This works only for ball game piece

                double sideLengthPixels = Math.sqrt(areaPixels);

                double angularWidthRad = sideLengthPixels / config.getWidthPixels() * config.getHorizontalViewingAngleRad();

                double horizontalOffsetMetersPerAngleRad = gamePieceType.getRadiusMeters() * 2 / angularWidthRad;
                double yDisplacementMeters = yaw.getRadians() * horizontalOffsetMetersPerAngleRad;
                double xyDistanceMeters = yDisplacementMeters / Math.sin(yaw.getRadians());
                double xDisplacementMeters = xyDistanceMeters * Math.cos(yaw.getRadians());

                double angularHeightRad = sideLengthPixels / config.getHeightPixels() * config.getVerticalViewingAngleRad();
                double verticalOffsetPerAngleMeters = gamePieceType.getRadiusMeters() * 2 / angularHeightRad;
                double zDisplacementMeters = pitch.getRadians() * verticalOffsetPerAngleMeters;

                Translation3d robotRelativeTranslation = new Translation3d(xDisplacementMeters, yDisplacementMeters, zDisplacementMeters);

                // TODO: add camera rotation and translation
                Translation3d fieldRelativeTranslation = robotPose.getTranslation().plus(cameraOffset).plus(robotRelativeTranslation);
                Pose3d fieldRelativePose = new Pose3d(fieldRelativeTranslation, robotPose.getRotation());
                gamePiecePose3d.add(fieldRelativePose);
                System.out.println(yaw.getDegrees() + " " + pitch.getDegrees() + " " + areaPixels);
                index += 1;
            }
        }
        return robotPose.nearest(gamePiecePose3d);
    }

    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        inputs.nearestPose = getNearestGamePiece();
    }
}
