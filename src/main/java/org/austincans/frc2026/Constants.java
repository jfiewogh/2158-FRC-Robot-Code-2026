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

package org.austincans.frc2026;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean isSim = currentMode == Mode.SIM;

  public static final double maxVoltage = 12;
  public static final double loopPeriodWatchdogSecs = 0.03;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY,

    /** Running a real robot with testing configurations */
    TEST
  }

  public static enum ControlMode {
    JOYSTICK,
    KEYBOARD
  }

  public static final ControlMode controlMode = ControlMode.JOYSTICK;

  public static final double loopPeriodSecs = 0.02;
  private static RobotType robotType = RobotBase.isReal() ? RobotType.COMPBOT : RobotType.SIMBOT;
  public static final boolean tuningMode = true;

  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT, DEMOBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum RobotType {
    SIMBOT,
    DEVBOT,
    DEMOBOT,
    COMPBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robotType == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robotType != RobotType.COMPBOT || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }

  public enum ControllerType {
    KEYBOARD,
    XBOX;
  }

  public static final ControllerType kControllerType = ControllerType.XBOX;

  /*
   * Vision Constants
   * These constants are used to define the vision system configuration,
   * such as the camera height, field layout, and camera calibration.
   */
  public class VisionConstants {
    public static final Pose3d kCameraPose =
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    public static AprilTagFieldLayout fieldLayout;

    // Cam mounted facing forward, half a meter forward of center, half a meter up from center,
    // pitched upward.
    private static final double camPitch = edu.wpi.first.math.util.Units.degreesToRadians(30.0);
    public static final Transform3d kRobotToCam =
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, -camPitch, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    /** Constants for an individual camera. */
    public static final record CameraConstants(
        String name, // Name of the camera
        int height, // Height of the camera in pixels
        int width, // Width of the camera in pixels
        Rotation2d horizontalFieldOfView, // Horizontal field of view of the camera
        Frequency framesPerSecond, // Frames per second of the camera
        Time latencyAvg, // Average latency of the camera
        Time latencyStdDev, // Standard deviation of latency
        double calibErrorAvg, // Average calibration error
        double calibErrorStdDev, // Standard deviation of calibration error
        Transform3d robotToCamera, // Transform from the camera to the robot
        double offset // Offset of the camera from the robot center
        ) {
      public double centerPixelYawRads() {
        return robotToCamera.getRotation().getZ();
      }
    }

    // The cameras on the robot. The first camera is the primary camera, and the second camera
    // is the secondary camera. The secondary camera is not used in the current implementation.
    public static final CameraConstants[] cameras =
        new CameraConstants[] {
          new CameraConstants(
              "cam0",
              800,
              1280,
              Rotation2d.fromDegrees(80),
              edu.wpi.first.units.Units.Hertz.of(20),
              edu.wpi.first.units.Units.Seconds.of(0.3),
              edu.wpi.first.units.Units.Seconds.of(0.02),
              0.8,
              0.08,
              new Transform3d(
                  new Translation3d(
                      edu.wpi.first.math.util.Units.inchesToMeters(11),
                      -edu.wpi.first.math.util.Units.inchesToMeters(12),
                      edu.wpi.first.math.util.Units.inchesToMeters(10)),
                  new Rotation3d(Math.PI, 0, 0)),
              edu.wpi.first.math.util.Units.inchesToMeters(1.6)),
          // Adding a second camera, however never installed :-(
          new CameraConstants(
              "cam1",
              600,
              800,
              Rotation2d.fromDegrees(60),
              edu.wpi.first.units.Units.Hertz.of(40),
              edu.wpi.first.units.Units.Seconds.of(0.15),
              edu.wpi.first.units.Units.Seconds.of(0.02),
              0.27,
              0.08,
              new Transform3d(
                  new Translation3d(
                      edu.wpi.first.math.util.Units.inchesToMeters(9.2),
                      edu.wpi.first.math.util.Units.inchesToMeters(13.5),
                      edu.wpi.first.math.util.Units.inchesToMeters(10)),
                  new Rotation3d(0, 0, -edu.wpi.first.math.util.Units.degreesToRadians(45))),
              edu.wpi.first.math.util.Units.inchesToMeters(0.0))
        };

    public static final double zMargin = 0.75;
    public static final double fieldBorderMargin = 0.5;
  }
}
