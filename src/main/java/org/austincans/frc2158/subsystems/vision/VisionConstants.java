// Copyright (c) 2025 the ausTIN CANs
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
// Updated on Wednesday November 05, 2025 at 17:57:47 CST

package org.austincans.frc2158.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import lombok.Builder;
import org.austincans.frc2158.Constants;
import org.austincans.frc2158.Constants.Mode;
import org.austincans.frc2158.util.LoggedTunableNumber;

public class VisionConstants {
  private static final boolean forceEnableInstanceLogging = false;
  public static final boolean enableInstanceLogging =
      forceEnableInstanceLogging || Constants.getMode() == Mode.REPLAY;

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;
  public static final double demoTagPosePersistenceSecs = 0.5;
  public static final double coralDetectConfidenceThreshold = 0.35;
  public static final double algaeDetectConfidenceThreshold = 0.35;
  public static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", 0.0);

  private static int monoExposure = 2200;
  private static double monoGain = 17.5;
  private static double monoDenoise = 1.0;
  private static int colorExposure = 4500;
  private static double colorGain = 5.0;

  public static LoggedTunableNumber[] pitchAdjustments =
      switch (Constants.getRobot()) {
        case DEVBOT ->
            new LoggedTunableNumber[] {
              new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust1", 0.0)
            };
        case COMPBOT ->
            new LoggedTunableNumber[] {
              new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust1", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust2", 0.0),
            };
        default -> new LoggedTunableNumber[] {};
      };
  public static CameraConfig[] cameras =
      switch (Constants.getRobot()) {
        case DEVBOT ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              0.254,
                              0.2032,
                              0.21113,
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-25.0 + pitchAdjustments[0].get()),
                                  Units.degreesToRadians(-20.0))))
                  .id("40265450")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .stdDevFactor(1.0)
                  .build(),
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              0.254,
                              -0.2032,
                              0.21113,
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-25.0 + pitchAdjustments[1].get()),
                                  Units.degreesToRadians(20.0))))
                  .id("40265453")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .stdDevFactor(1.0)
                  .build()
            };
        case COMPBOT ->
            new CameraConfig[] {
              // Front Left
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              0.2794,
                              0.2286,
                              0.21113,
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-25.0 + pitchAdjustments[0].get()),
                                  Units.degreesToRadians(-20.0))))
                  .id("40530395")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .build(),
              // Front Right
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              0.2794,
                              -0.2286,
                              0.21113,
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-25.0 + pitchAdjustments[1].get()),
                                  Units.degreesToRadians(20.0))))
                  .id("40552081")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .build(),
              // Back Up
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              Units.inchesToMeters(3.209),
                              Units.inchesToMeters(-10),
                              Units.inchesToMeters(26.485),
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(25.0 + pitchAdjustments[2].get()),
                                  Units.degreesToRadians(163.5))))
                  .id("24737133")
                  .width(1280)
                  .height(960)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .stdDevFactor(1.25)
                  .build(),
            };
        default -> new CameraConfig[] {};
      };

  @Builder
  public record CameraConfig(
      Supplier<Pose3d> pose,
      String id,
      int width,
      int height,
      int autoExposure,
      int exposure,
      double gain,
      double denoise,
      double stdDevFactor) {}

  private VisionConstants() {}
}
