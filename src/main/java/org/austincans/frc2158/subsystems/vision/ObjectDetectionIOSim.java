package org.austincans.frc2158.subsystems.vision;

import org.ironmaple.simulation.SimulatedArena3D;

public class ObjectDetectionIOSim implements ObjectDetectionIO {
    private final String gamePieceType;

    public ObjectDetectionIOSim(String gamePieceType) {
        this.gamePieceType = gamePieceType;
    }
    
    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        inputs.gamePiecePoses = SimulatedArena3D.getInstance().getGamePiecePosesArrayByType(gamePieceType);
    }
}
