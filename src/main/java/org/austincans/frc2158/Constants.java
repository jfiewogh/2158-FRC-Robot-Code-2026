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
// Updated on Wednesday February 18, 2026 at 20:17:49 CST

package org.austincans.frc2158;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import org.austincans.frc2158.util.ConfigUtil;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double maxVoltage = 12;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

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

  public static class OperatorConstants {
    public static final int kDriverControllerPort =
        ConfigUtil.getConfigInt("DRIVER_CONTROLLER_PORT", 0);
  }

  public static class RobotConstants {
    public static double kBumperWidthMeters =
        Units.inchesToMeters(ConfigUtil.getConfigDouble("ROBOT_BUMPER_FRONT_INCHES", 28));
    public static double kBumperLengthMeters =
        Units.inchesToMeters(ConfigUtil.getConfigDouble("ROBOT_BUMPER_RIGHT_INCHES", 28));
  }

  public static final double kWheelRadius =
      Units.inchesToMeters(ConfigUtil.getConfigDouble("ROBOT_WHEEL_RADIUS_INCHES", 2.0));

  public enum ControllerType {
    KEYBOARD,
    XBOX;
  }

  public static final ControllerType kControllerType = ControllerType.XBOX;

  public static class PIDConstants {
    // 1 means at the farthest possible point, it is at max speed
    // be careful with D

    public static final PIDController kElevatorController = new PIDController(10, 0, 0);
  }

  public static enum Mechanism {
    kDrive(1.0 / 5.0),
    kAngle(1.0 / 3.0 / 4.0),
    kElevator(1.0 / 50.0); // random value

    private final double gearRatio;

    private Mechanism(double gearRatio) {
      this.gearRatio = gearRatio;
    }

    public double toMechanism(double value) {
      return value * gearRatio;
    }

    public double fromMechanism(double value) {
      return value / gearRatio;
    }
  }

  public static enum MotorSpeed {
    kKraken(5800),
    kVortex(6784);

    private final double freeSpeedRotationsPerMinute;
    private final double freeSpeedRotationsPerSecond;

    private MotorSpeed(double freeSpeedRotationsPerMinute) {
      this.freeSpeedRotationsPerMinute = freeSpeedRotationsPerMinute;
      freeSpeedRotationsPerSecond = freeSpeedRotationsPerMinute / 60;
    }

    public double getFreeSpeedRotationsPerMinute() {
      return freeSpeedRotationsPerMinute;
    }

    public double getFreeSpeedRotationsPerSecond() {
      return freeSpeedRotationsPerSecond;
    }
  }

  public static enum MechanismSpeed {
    kDrive(MotorSpeed.kKraken, Mechanism.kDrive),
    kAngle(MotorSpeed.kKraken, Mechanism.kAngle),
    kElevator(MotorSpeed.kVortex, Mechanism.kElevator);

    private final MotorSpeed motorSpeed;
    private final Mechanism mechanism;

    private MechanismSpeed(MotorSpeed motorSpeed, Mechanism mechanism) {
      this.motorSpeed = motorSpeed;
      this.mechanism = mechanism;
    }

    public double getMaxMechanismSpeedRotationsPerSecond() {
      return mechanism.toMechanism(motorSpeed.getFreeSpeedRotationsPerSecond());
    }

    public double getMaxMechanismSpeedRadiansPerSecond() {
      return Units.rotationsToRadians(getMaxMechanismSpeedRotationsPerSecond());
    }
  }

  public static AprilTag[] sourceAprilTags = {
    // opponent
    AprilTag.k1,
    AprilTag.k2,
    // alliance
    AprilTag.k12,
    AprilTag.k13
  };

  public static enum ReefAprilTag {
    // opponent side
    k6(530.49, 130.17, 12.13, 300, 0),
    k7(546.87, 158.50, 12.13, 0, 0),
    k8(530.49, 186.83, 12.13, 60, 0),
    k9(497.77, 186.83, 12.13, 120, 0),
    k10(481.39, 158.50, 12.13, 180, 0),
    k11(497.77, 130.17, 12.13, 240, 0),

    k17(160.39, 130.17, 12.13, 240, 0),
    k18(144.00, 158.50, 12.13, 180, 0),
    k19(160.39, 186.83, 12.13, 120, 0),
    k20(193.10, 186.83, 12.13, 60, 0),
    k21(209.49, 158.50, 12.13, 0, 0),
    k22(193.10, 130.17, 12.13, 300, 0);

    private final Pose3d pose;
    private final Pose2d leftAlignPose;
    private final Pose2d rightAlignPose;

    private ReefAprilTag(double x, double y, double z, double zRotation, double yRotation) {
      pose =
          new Pose3d(
              Units.inchesToMeters(x),
              Units.inchesToMeters(y),
              Units.inchesToMeters(z),
              new Rotation3d(
                  0, Units.degreesToRadians(yRotation), Units.degreesToRadians(zRotation)));

      // I don't know why these values are flipped
      leftAlignPose =
          pose.toPose2d()
              .plus(
                  new Transform2d(
                      RobotConstants.kBumperLengthMeters / 2, -0.16, Rotation2d.k180deg));
      rightAlignPose =
          pose.toPose2d()
              .plus(
                  new Transform2d(
                      RobotConstants.kBumperLengthMeters / 2, 0.16, Rotation2d.k180deg));
    }

    public Pose3d getPose() {
      return pose;
    }

    public Pose2d getLeftAlignPose() {
      return leftAlignPose;
    }

    public Pose2d getRightAlignPose() {
      return rightAlignPose;
    }
  }

  public enum BargeAprilTag {}

  public enum SourceAprilTag {}

  public enum ProcesserAprilTag {}

  public enum AprilTag {
    // values in inches

    k1(657.37, 25.80, 58.50, 126, 0),
    k2(657.37, 291.20, 58.50, 234, 0),
    k12(0, 0, 0, 0, 0),
    k13(0, 0, 0, 0, 0),

    k3(0, 0, 0, 0, 0),
    k16(0, 0, 0, 0, 0),

    k4(0, 0, 0, 0, 30),
    k5(0, 0, 0, 0, 30),
    k14(0, 0, 0, 0, 30),
    k15(0, 0, 0, 0, 30);

    private final Pose3d pose;

    private AprilTag(double x, double y, double z, double zRotation, double yRotation) {

      pose =
          new Pose3d(
              Units.inchesToMeters(x),
              Units.inchesToMeters(y),
              Units.inchesToMeters(z),
              new Rotation3d(
                  0, Units.degreesToRadians(yRotation), Units.degreesToRadians(zRotation)));
    }

    public Pose3d getPose() {
      return pose;
    }
  }

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
