package org.austincans.frc2158.subsystems.vision;

import java.util.ArrayList;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightResults;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;
import org.austincans.frc2158.subsystems.vision.ObjectDetection.GamePieceType;
import org.austincans.frc2158.subsystems.vision.VisionConstants.LimelightConfig;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ObjectDetectionIOReal implements ObjectDetectionIO {
    private final String limelightName;
    private final GamePieceType gamePieceType;
    private final LimelightConfig config;

    // TODO: add to config
    private static final Pose3d robotRelativeCameraPose = new Pose3d(
        new Translation3d(0, 0, 0), // meters
        new Rotation3d(0, 0, 0));

    public ObjectDetectionIOReal(String limelightName, GamePieceType gamePieceType, LimelightConfig config) {
        this.limelightName = limelightName;
        this.gamePieceType = gamePieceType;
        this.config = config;
    }

    // Only works for ball game piece
    // TODO: Add version for other game pieces (coral, note)
    private Translation3d getTranslationFromDetectorResult(LimelightTarget_Detector result) {
        Rotation2d yaw = Rotation2d.fromDegrees(result.tx);
        Rotation2d pitch = Rotation2d.fromDegrees(result.ty);

        double areaPixels = result.ta * config.getWidthPixels() * config.getHeightPixels();
        double sideLengthPixels = Math.sqrt(areaPixels);

        double angularWidthRad = sideLengthPixels / config.getWidthPixels() * config.getHorizontalViewingAngleRad();

        double horizontalOffsetMetersPerAngleRad = gamePieceType.getRadiusMeters() * 2 / angularWidthRad;
        double yDisplacementMeters = yaw.getRadians() * horizontalOffsetMetersPerAngleRad;
        double xyDistanceMeters = yDisplacementMeters / Math.sin(yaw.getRadians());
        double xDisplacementMeters = xyDistanceMeters * Math.cos(yaw.getRadians());

        double angularHeightRad = sideLengthPixels / config.getHeightPixels() * config.getVerticalViewingAngleRad();
        double verticalOffsetPerAngleMeters = gamePieceType.getRadiusMeters() * 2 / angularHeightRad;
        double zDisplacementMeters = pitch.getRadians() * verticalOffsetPerAngleMeters;

        return new Translation3d(xDisplacementMeters, yDisplacementMeters, zDisplacementMeters);
    }

    private Pose3d getFieldRelativeCameraPose(Pose3d robotPose) {
        return new Pose3d(
            robotPose.getTranslation().plus(robotRelativeCameraPose.getTranslation()),
            robotPose.getRotation().plus(robotRelativeCameraPose.getRotation()));
    }

    private ArrayList<Pose3d> getFieldRelativeGamePiecePoses() {
        LimelightResults latestResults = LimelightHelpers.getLatestResults(limelightName);
        LimelightTarget_Detector[] detectorResults = latestResults.targets_Detector;

        Pose3d robotPose = LimelightHelpers.getBotPose3d(limelightName);

        ArrayList<Pose3d> gamePiecePoses = new ArrayList<Pose3d>();
        
        for (LimelightTarget_Detector result : detectorResults) {
            if (result.className == gamePieceType.getClassName()) {
                Translation3d cameraRelativeGamePieceTranslation = getTranslationFromDetectorResult(result);

                Pose3d fieldRelativeCameraPose = getFieldRelativeCameraPose(robotPose);
                
                Pose3d fieldRelativeGamePiecePose = fieldRelativeCameraPose.plus(
                    new Transform3d(cameraRelativeGamePieceTranslation, Rotation3d.kZero));
                gamePiecePoses.add(fieldRelativeGamePiecePose);
            }
        }
        return gamePiecePoses;
    }

    private Pose3d getNearestGamePiece() {
        Pose3d robotPose = LimelightHelpers.getBotPose3d(limelightName);
        return robotPose.nearest(getFieldRelativeGamePiecePoses());
    }

    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        inputs.nearestPose = getNearestGamePiece();
    }
}
