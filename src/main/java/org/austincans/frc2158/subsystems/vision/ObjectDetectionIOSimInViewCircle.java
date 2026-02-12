package org.austincans.frc2158.subsystems.vision;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.austincans.frc2158.subsystems.vision.ObjectDetectionIO.ObjectDetectionIOInputs;
import org.ironmaple.simulation.SimulatedArena3D;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class ObjectDetectionIOSimInViewCircle {
    public enum SimCameraConfig {
        LIMELIGHT_3(62.5, 48.9),
        LIMELIGHT_3A(54.5, 42),
        LIMELIGHT_3G(82, 56.2),
        LIMELIGHT_4(82, 56.2);

        private double horizontalFOVDeg;
        private double verticalFOVDeg; 

        private SimCameraConfig(double horizontalFOVDeg, double verticalFOVDeg) {
            this.horizontalFOVDeg = horizontalFOVDeg;
            this.verticalFOVDeg = verticalFOVDeg;
        }

        public double getHorizontalFOVRad() {
            return Units.degreesToRadians(horizontalFOVDeg);
        }

        public double getVerticalFOVRad() {
            return Units.degreesToRadians(verticalFOVDeg);
        }
    }

    private final String gamePieceType;
    private final SimCameraConfig config;
    private final Supplier<Pose3d> fieldRelRobotPoseSupplier;
    private final Supplier<Pose3d> robotRelCameraPoseSupplier;

    public ObjectDetectionIOSimInViewCircle(String gamePieceType, SimCameraConfig config, Supplier<Pose3d> fieldRelRobotPoseSupplier, Supplier<Pose3d> robotRelCameraPoseSupplier) {
        this.gamePieceType = gamePieceType;
        this.config = config;
        this.fieldRelRobotPoseSupplier = fieldRelRobotPoseSupplier;
        this.robotRelCameraPoseSupplier = robotRelCameraPoseSupplier;
    }

    private Pose3d[] getAllGamePiecePoses() {
        return SimulatedArena3D.getInstance().getGamePiecePosesArrayByType(gamePieceType);
    }

    private boolean checkPoseInView(Pose3d fieldRelGamePiecePose) {
        Pose3d robotRelCameraPose = robotRelCameraPoseSupplier.get();
        Pose3d fieldRelCameraPose = fieldRelRobotPoseSupplier.get().plus(new Transform3d(
            robotRelCameraPose.getTranslation(), robotRelCameraPose.getRotation()));
        Transform3d robotRelGamePieceTransform = fieldRelGamePiecePose.minus(fieldRelCameraPose);
        Translation3d robotRelGamePieceTranslation = robotRelGamePieceTransform.getTranslation();

        double x = robotRelGamePieceTranslation.getX();
        double y = robotRelGamePieceTranslation.getY();
        double z = robotRelGamePieceTranslation.getZ();

        double yMaxRadius = x * Math.sin(config.getHorizontalFOVRad());
        double zMaxRadius = x * Math.sin(config.getVerticalFOVRad());

        return Math.abs(y) < yMaxRadius && Math.abs(z) < zMaxRadius;
    }

    private Pose3d[] getGamePiecePosesInView() {
        ArrayList<Pose3d> gamePiecePoses = new ArrayList<Pose3d>();

        for (Pose3d fieldRelGamePiecePose : getAllGamePiecePoses()) {
            if (checkPoseInView(fieldRelGamePiecePose)) {
                gamePiecePoses.add(fieldRelGamePiecePose);
            }
        }
        return (Pose3d[]) gamePiecePoses.toArray();
    }

    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        inputs.gamePiecePoses = getGamePiecePosesInView();
    }
}
