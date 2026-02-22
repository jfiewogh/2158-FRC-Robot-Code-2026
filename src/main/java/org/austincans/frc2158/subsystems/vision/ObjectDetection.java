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
// Updated on Saturday February 21, 2026 at 23:31:39 CST

package org.austincans.frc2158.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ObjectDetection extends SubsystemBase {
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
      this.radiusMeters = radiusMeters;
    }

    public String getClassName() {
      return className;
    }

    public double getRadiusMeters() {
      return radiusMeters;
    }
  }

  private final ODIO io;
  private final ObjectDetectionIOInputsAutoLogged inputs = new ObjectDetectionIOInputsAutoLogged();

  public ObjectDetection(ODIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ObjectDetection", inputs);
  }

  public Pose3d getNearestGamePiecePose() {
    return inputs.nearestGamePiecePose;
  }

  public Supplier<Pose3d> getNearestGamePiecePoseSupplier() {
    return this::getNearestGamePiecePose;
  }

  public Pose3d[] getGamePiecePoses() {
    return inputs.gamePiecePoses;
  }
}
