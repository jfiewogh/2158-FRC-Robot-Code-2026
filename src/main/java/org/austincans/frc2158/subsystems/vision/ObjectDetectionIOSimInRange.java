package org.austincans.frc2158.subsystems.vision;

import java.util.ArrayList;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena3D;
import edu.wpi.first.math.geometry.Pose3d;

public class ObjectDetectionIOSimInRange implements ObjectDetectionIO {
    private final String gamePieceType;
    private final double maxDistanceMeters;
    private final Supplier<Pose3d> robotPoseSupplier;

    public ObjectDetectionIOSimInRange(String gamePieceType, double maxDistanceMeters, Supplier<Pose3d> robotPoseSupplier) {
        this.gamePieceType = gamePieceType;
        this.maxDistanceMeters = maxDistanceMeters;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    private Pose3d[] getAllGamePiecePoses() {
        return SimulatedArena3D.getInstance().getGamePiecePosesArrayByType(gamePieceType);
    }

    private Pose3d[] getGamePiecePosesInRadius() {
        Pose3d robotPose = robotPoseSupplier.get();

        ArrayList<Pose3d> gamePiecePoses = new ArrayList<Pose3d>();
        for (Pose3d gamePiecePose : getAllGamePiecePoses()) {
            double distance = robotPose.getTranslation().getDistance(gamePiecePose.getTranslation());
            if (distance < maxDistanceMeters) {
                gamePiecePoses.add(gamePiecePose);
            }
        }
        return (Pose3d[]) gamePiecePoses.toArray();
    }
    
    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        inputs.gamePiecePoses = getGamePiecePosesInRadius();
    }
}
