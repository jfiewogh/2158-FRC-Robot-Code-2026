// Copyright (c) 2026 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// oooooooooooo ooooooooo.     .oooooo.        ooooooooo.              .o8                     .   
// `888'     `8 `888   `Y88.  d8P'  `Y8b       `888   `Y88.           "888                   .o8   
//  888          888   .d88' 888                888   .d88'  .ooooo.   888oooo.   .ooooo.  .o888oo 
//  888oooo8     888ooo88P'  888                888ooo88P'  d88' `88b  d88' `88b d88' `88b   888   
//  888    "     888`88b.    888                888`88b.    888   888  888   888 888   888   888   
//  888          888  `88b.  `88b    ooo        888  `88b.  888   888  888   888 888   888   888 . 
// o888o        o888o  o888o  `Y8bood8P'       o888o  o888o `Y8bod8P'  `Y8bod8P' `Y8bod8P'   "888" 
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//

package org.austincans.frc2158;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.HashMap;

// Singleton class
public class GameTrack {
  private static GameTrack instance;

  public static GameTrack getInstance() {
    if (instance == null) {
      instance = new GameTrack();
    }
    return instance;
  }

  public enum GamePieceType {
    // 2025
    CORAL,
    ALGAE,
    // 2026
    FUEL
  }

  public static class GamePiece {
    private Pose3d pose;
    private boolean isScored;

    public GamePiece(Pose3d pose, boolean isScored) {
      this.pose = pose;
      this.isScored = isScored;
    }

    public GamePiece(Pose3d pose) {
      this(pose, false);
    }

    public Pose3d getPose() {
      return pose;
    }

    public double getDistance(Pose3d otherPose) {
      return pose.getTranslation().getDistance(otherPose.getTranslation());
    }

    public boolean isScored() {
      return isScored;
    }
  }

  private static HashMap<GamePieceType, GamePiece[]> gamePiecesMap;

  // Getters

  public static GamePiece[] getGamePiecesOfType(GamePieceType type) {
    return gamePiecesMap.get(type);
  }

  public static GamePiece getClosestUnscoredGamePieceOfType(
      GamePieceType gamePieceType, Pose3d subjectPose) {
    GamePiece[] gamePiecesOfType = getGamePiecesOfType(gamePieceType);
    GamePiece closestGamePiece = gamePiecesOfType[0];
    for (int i = 1; i < gamePiecesOfType.length; i++) {
      GamePiece gamePiece = gamePiecesOfType[i];
      if (!gamePiece.isScored()
          && gamePiece.getDistance(subjectPose) < closestGamePiece.getDistance(subjectPose)) {
        closestGamePiece = gamePiece;
      }
    }
    return closestGamePiece;
  }

  // Setters

  public static void setGamePiecesOfTypeFromPoses(GamePieceType gamePieceType, Pose3d[] poses) {
    GamePiece[] gamePieces = new GamePiece[poses.length];
    for (int i = 0; i < poses.length; i++) {
      gamePieces[i] = new GamePiece(poses[i]);
    }
    setGamePiecesOfType(gamePieceType, gamePieces);
  }

  public static void setGamePiecesOfType(GamePieceType gamePieceType, GamePiece[] gamePieces) {
    gamePiecesMap.put(gamePieceType, gamePieces);
  }

  public static void addScoredGamePieceOfType() {}
}
