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
// Updated on Saturday February 21, 2026 at 16:10:33 CST

package org.austincans.frc2158.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena3D;

public class ODIOSimAll implements ODIO {
  private final String gamePieceType;
  private final Supplier<Pose3d> fieldRelRobotPoseSupplier;

  public ODIOSimAll(String gamePieceType, Supplier<Pose3d> fieldRelRobotPoseSupplier) {
    this.gamePieceType = gamePieceType;
    this.fieldRelRobotPoseSupplier = fieldRelRobotPoseSupplier;
  }

  private Pose3d[] getAllGamePiecePoses() {
    return SimulatedArena3D.getInstance().getGamePiecePosesArrayByType(gamePieceType);
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.gamePiecePoses = getAllGamePiecePoses();
    ArrayList<Pose3d> gamePiecePosesArrayList =
        new ArrayList<Pose3d>(Arrays.asList(inputs.gamePiecePoses));
    inputs.nearestGamePiecePose =
        inputs.gamePiecePoses.length > 0
            ? fieldRelRobotPoseSupplier.get().nearest(gamePiecePosesArrayList)
            : null;
  }
}
