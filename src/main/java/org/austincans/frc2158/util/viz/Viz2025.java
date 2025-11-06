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

package org.austincans.frc2158.util.viz;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

/**
 * Visualization of the 2025 Robot
 *
 * @author Austin Cans Robotics Team 2158
 */
public class Viz2025 implements Drawable {

  // We use Colson Performa 4x1-1/2 wheels, which are 4 inches in diameter.
  // The wheelbase and track width are 24.229 inches, which is the same as the
  // 2024 robot. The wheelbase is the distance between the front and back wheels,
  // and the track width is the distance between the left and right wheels.
  public static final Distance trackWidth = Inches.of(24.229);
  public static final Distance wheelBase = Inches.of(24.229);
  public static final Distance wheelDiameter = Inches.of(4.0);
  public static final Distance wheelCircumference = wheelDiameter.times(Math.PI);

  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase.in(Meters) / 2, trackWidth.in(Meters) / 2),
        new Translation2d(wheelBase.in(Meters) / 2, -trackWidth.in(Meters) / 2),
        new Translation2d(-wheelBase.in(Meters) / 2, trackWidth.in(Meters) / 2),
        new Translation2d(-wheelBase.in(Meters) / 2, -trackWidth.in(Meters) / 2)
      };

  @SuppressWarnings("unused")
  private final FieldViz fieldViz;

  private final String prefix;
  private double elevatorHeight = 0.0;
  private double algaeAngle = 0.0;

  @SuppressWarnings("unused")
  private double algaeAngularVelocity = 0.0;

  @SuppressWarnings("unused")
  private boolean algaeDropped = false;

  private Pose2d estimatedPose;
  private Pose2d actualPose = new Pose2d();
  private final Pose3d[] mechanisms = new Pose3d[11];
  private boolean hasCoral = true;
  private boolean hasAlgae = true;
  private static final Transform3d coralOffset =
      new Transform3d(
          new Translation3d(0.236157, -0.157444, 0.383131),
          new Rotation3d(0, Math.toRadians(42), 0));
  private static final Transform3d algaeOffset =
      new Transform3d(new Translation3d(0.389977, 0, 0.602934), Rotation3d.kZero);

  private static final int CLIMBER_ID = 0;
  private static final int STAGE2_ID = 1;
  private static final int STAGE3_ID = 2;
  private static final int STAGE4_ID = 3;
  private static final int ALGAE_ID = 4;
  private static final int BL_ID = 5;
  private static final int BR_ID = 6;
  private static final int FL_ID = 7;
  private static final int FR_ID = 8;
  private static final int RED_BUMPER = 9;
  private static final int BLUE_BUMPER = 10;

  private final Pose3d useBumper = new Pose3d(new Translation3d(0, 0, 1), Rotation3d.kZero);
  private final Pose3d invisible = new Pose3d(new Translation3d(0, 0, -100), Rotation3d.kZero);

  /** Visualization of the 2025 Robot */
  public Viz2025(FieldViz fieldViz, String prefix) {
    this.fieldViz = fieldViz;
    this.prefix = prefix;
    for (int i = 0; i < mechanisms.length; i++) {
      mechanisms[i] = new Pose3d();
    }
    this.reset(Pose2d.kZero);
  }

  /** Set the angle of the climber, with 0 being straight up and down. */
  public void setClimberAngle(Angle angle) {
    mechanisms[CLIMBER_ID] =
        new Pose3d(
            new Translation3d(-0.280502, 0, 0.461962),
            new Rotation3d(Radians.of(0.0), angle.div(300), Radians.of(0.0)));
  }

  /** Show coral in the coral scorer. */
  public void setHasCoral(boolean hasCoral) {
    this.hasCoral = hasCoral;
  }

  /** Show algae in the algae scorer. */
  public void setHasAlgae(boolean hasAlgae) {
    this.hasAlgae = hasAlgae;
  }

  /** Set the estimated drivetrain state, including swerve drive poses.` */
  public void setDrivetrainState(Pose2d estimatedPose, Rotation2d[] states) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        mechanisms[RED_BUMPER] = useBumper;
        mechanisms[BLUE_BUMPER] = invisible;
      } else {
        mechanisms[BLUE_BUMPER] = useBumper;
        mechanisms[RED_BUMPER] = invisible;
      }
    } else {
      mechanisms[BLUE_BUMPER] = invisible;
      mechanisms[RED_BUMPER] = useBumper;
    }
    this.estimatedPose = estimatedPose;
    if (RobotBase.isReal()) {
      this.actualPose = estimatedPose;
    }
    mechanisms[BL_ID] =
        new Pose3d(new Translation3d(moduleTranslations[0]), new Rotation3d(states[0]));
    mechanisms[BR_ID] =
        new Pose3d(new Translation3d(moduleTranslations[1]), new Rotation3d(states[1]));
    mechanisms[FL_ID] =
        new Pose3d(new Translation3d(moduleTranslations[2]), new Rotation3d(states[2]));
    mechanisms[FR_ID] =
        new Pose3d(new Translation3d(moduleTranslations[3]), new Rotation3d(states[3]));
  }

  /** Set the actual pose (only different for sim) */
  public void setActualPose(Pose2d pose) {
    this.actualPose = pose;
  }

  /** Set the elevator height relative to its lowest point. */
  public void setElevatorHeight(Distance height) {
    elevatorHeight = height.in(Meters);
  }

  /** Reset all values, getting ready for auto. */
  public void reset(Pose2d pose) {
    elevatorHeight = 0.0;
    algaeAngle = 0.0;
    algaeDropped = false;
    estimatedPose = pose;
    actualPose = pose;
    mechanisms[CLIMBER_ID] = new Pose3d(Translation3d.kZero, Rotation3d.kZero);
  }

  /** Publish all values to Logger */
  @Override
  public void drawImpl() {
    algaeAngle = 90.0;
    mechanisms[STAGE2_ID] =
        new Pose3d(
            new Translation3d(0, 0, Math.min(elevatorHeight, 0.862013 - 0.173028)),
            Rotation3d.kZero);
    mechanisms[STAGE3_ID] =
        new Pose3d(
            new Translation3d(0, 0, Math.min(elevatorHeight, 1.57956 - 0.199172)),
            Rotation3d.kZero);
    mechanisms[STAGE4_ID] =
        new Pose3d(
            new Translation3d(0, 0, Math.min(elevatorHeight, 1.96337 - 0.226559)),
            Rotation3d.kZero);
    Rotation3d algaeRotation3d = new Rotation3d(0.0, Units.degreesToRadians(algaeAngle), 0.0);
    Translation3d algaeTranslation3d =
        new Translation3d(
            0.13335, 0.006358, 0.615387 + Math.min(elevatorHeight, 1.96337 - 0.226559));
    mechanisms[ALGAE_ID] = new Pose3d(algaeTranslation3d, algaeRotation3d);
    Logger.recordOutput(prefix + "Viz/Mechanisms", mechanisms);
    Logger.recordOutput(prefix + "Viz/EstimatedPose", estimatedPose);
    Logger.recordOutput(prefix + "Viz/ActualPose", actualPose);
    if (hasCoral) {
      Logger.recordOutput(
          prefix + "Viz/HeldCoral",
          new Pose3d(actualPose).plus(mechanisms[STAGE4_ID].minus(Pose3d.kZero)).plus(coralOffset));
    } else {
      Logger.recordOutput(prefix + "Viz/HeldCoral", invisible);
    }
    if (hasAlgae) {
      Logger.recordOutput(
          prefix + "Viz/HeldAlgae",
          new Pose3d(actualPose).plus(mechanisms[STAGE4_ID].minus(Pose3d.kZero)).plus(algaeOffset));
    } else {
      Logger.recordOutput(prefix + "Viz/HeldAlgae", invisible);
    }
  }

  public void setAlgaeAngle(Angle angle) {
    algaeAngle = angle.in(Degrees);
  }
}
