package org.austincans.frc2158.subsystems.vision;

import org.ironmaple.simulation.SimulatedArena3D;

import edu.wpi.first.math.geometry.Pose3d;

public class ObjectDetectionIOSimAll implements ObjectDetectionIO {
    private final String gamePieceType;

    public ObjectDetectionIOSimAll(String gamePieceType) {
        this.gamePieceType = gamePieceType;
    }

    private Pose3d[] getAllGamePiecePoses() {
        return SimulatedArena3D.getInstance().getGamePiecePosesArrayByType(gamePieceType);
    }
    
    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        inputs.gamePiecePoses = getAllGamePiecePoses();
    }
}
