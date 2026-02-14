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
// Updated on Saturday February 14, 2026 at 11:44:36 CST

package org.austincans.frc2158.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena3D;

// NOTE: This can see past obstacles on field
public class ObjectDetectionIOSimInViewCircle implements ObjectDetectionIO {
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
  private final double maxDetectionDistanceMeters;
  private final Supplier<Pose3d> fieldRelRobotPoseSupplier;
  private final Supplier<Pose3d> robotRelCameraPoseSupplier;

  public ObjectDetectionIOSimInViewCircle(
      String gamePieceType,
      SimCameraConfig config,
      double maxDetectionDistanceMeters,
      Supplier<Pose3d> fieldRelRobotPoseSupplier,
      Supplier<Pose3d> robotRelCameraPoseSupplier) {
    this.gamePieceType = gamePieceType;
    this.config = config;
    this.maxDetectionDistanceMeters = maxDetectionDistanceMeters;
    this.fieldRelRobotPoseSupplier = fieldRelRobotPoseSupplier;
    this.robotRelCameraPoseSupplier = robotRelCameraPoseSupplier;
  }

  private Pose3d[] getAllGamePiecePoses() {
    return SimulatedArena3D.getInstance().getGamePiecePosesArrayByType(gamePieceType);
  }

  private boolean checkPoseInView(Pose3d fieldRelGamePiecePose) {
    Pose3d robotRelCameraPose = robotRelCameraPoseSupplier.get();
    Pose3d fieldRelCameraPose =
        fieldRelRobotPoseSupplier
            .get()
            .plus(
                new Transform3d(
                    robotRelCameraPose.getTranslation(), robotRelCameraPose.getRotation()));
    Transform3d robotRelGamePieceTransform = fieldRelGamePiecePose.minus(fieldRelCameraPose);
    Translation3d robotRelGamePieceTranslation = robotRelGamePieceTransform.getTranslation();

    double x = robotRelGamePieceTranslation.getX();
    if (x > maxDetectionDistanceMeters) {
      return false;
    }
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
    return gamePiecePoses.toArray(new Pose3d[0]);
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.gamePiecePoses = getGamePiecePosesInView();
  }
}
