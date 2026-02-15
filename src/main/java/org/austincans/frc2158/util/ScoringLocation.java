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
// Updated on Sunday February 15, 2026 at 11:15:12 CST

package org.austincans.frc2158.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import org.austincans.frc2158.FieldConstants;
import org.littletonrobotics.junction.Logger;

/*************************************************
 * Scoring Locations for the 2025 game Reefscape *
 * @author Austin Cans Robotics Team 2158        *
 *************************************************/
public class ScoringLocation {

  /** Configuration object */
  public static final Distance P0 =
      Inches.of(ConfigUtil.getConfigDouble("ELEVATOR_DISTANCE_P0", 21.5)); // Algae 1

  public static final Distance P1 =
      Inches.of(ConfigUtil.getConfigDouble("ELEVATOR_DISTANCE_P1", 26.45)); // Coral l1
  public static final Distance P2 =
      Inches.of(ConfigUtil.getConfigDouble("ELEVATOR_DISTANCE_P2", 36.2)); // Algae 2
  public static final Distance P3 =
      Inches.of(ConfigUtil.getConfigDouble("ELEVATOR_DISTANCE_P3", 43.6)); // Coral L3
  public static final Distance P4 =
      Inches.of(ConfigUtil.getConfigDouble("ELEVATOR_DISTANCE_P4", 68.9)); // Coral L4
  public static final Distance P5 =
      Inches.of(ConfigUtil.getConfigDouble("ELEVATOR_DISTANCE_P5", 60.0)); // Barge

  /** Reef locations */
  public static enum CoralLocation {
    // @formatter:off
    A(coralPose(Rotation2d.kZero, true, 0, 0), 18),
    B(coralPose(Rotation2d.kZero, false, 0, 0), 18),
    C(coralPose(Rotation2d.fromDegrees(60), true, 0, 0), 17),
    D(coralPose(Rotation2d.fromDegrees(60), false, 0, 0), 17),
    E(coralPose(Rotation2d.fromDegrees(120), true, 0, 0), 22),
    F(coralPose(Rotation2d.fromDegrees(120), false, 0, 0), 22),
    G(coralPose(Rotation2d.k180deg, true, 0, 0), 21),
    H(coralPose(Rotation2d.k180deg, false, 0, 0), 21),
    I(coralPose(Rotation2d.fromDegrees(240), true, -1.5, 0), 20),
    J(coralPose(Rotation2d.fromDegrees(240), false, -2.5, 0), 20),
    K(coralPose(Rotation2d.fromDegrees(300), true, 0, 2), 19),
    L(coralPose(Rotation2d.fromDegrees(300), false, 0, 0), 19);
    // @formatter:on

    public final Pose2d pose;
    public final int tag;

    /** Get the pose of the coral location */
    private static Pose2d coralPose(
        Rotation2d direction, boolean offset, double rlOffset, double fbOffset) {
      Distance bumperFront =
          Inches.of(ConfigUtil.getConfigDouble("ROBOT_BUMPER_FRONT_INCHES", 17.5));
      return new Pose2d(
          FieldConstants.Reef.center
              .plus(
                  new Translation2d(
                      FieldConstants.Reef.inscribedRadius.in(Meters)
                          + bumperFront.in(Meters)
                          + Units.inchesToMeters(fbOffset),
                      direction.plus(Rotation2d.k180deg)))
              .plus(
                  new Translation2d(
                      Units.inchesToMeters((offset ? 12 : 0) + rlOffset),
                      direction.plus(Rotation2d.kCCW_90deg))),
          direction);
    }

    CoralLocation(Pose2d pose, int tag) {
      this.pose = pose;
      this.tag = tag;
      String name = this.toString();
      Logger.recordOutput("Reef/" + name, this.pose);
    }

    /** Deserialize from integer */
    public static CoralLocation fromInt(int id) {
      switch (id) {
        case 0:
          return CoralLocation.A;
        case 1:
          return CoralLocation.B;
        case 2:
          return CoralLocation.C;
        case 3:
          return CoralLocation.D;
        case 4:
          return CoralLocation.E;
        case 5:
          return CoralLocation.F;
        case 6:
          return CoralLocation.G;
        case 7:
          return CoralLocation.H;
        case 8:
          return CoralLocation.I;
        case 9:
          return CoralLocation.J;
        case 10:
          return CoralLocation.K;
        case 11:
          return CoralLocation.L;
        default:
          return null;
      }
    }
  }

  /** set of height modes */
  public enum Height {
    KP0("Algae 1", P0, true),
    KP1("Coral 1", P1, false),
    KP2("Algae 2", P2, true),
    KP3("Coral 3", P3, false),
    KP4("Coral 4", P4, false),
    KP5("Barge", P5, true);

    public final String displayName;
    public final Distance height;
    public final boolean isAlgae;

    Height(String displayName, Distance height, boolean isAlgae) {
      this.displayName = displayName;
      this.height = height;
      this.isAlgae = isAlgae;
    }

    /** Serialize to an integer */
    public int toInt() {
      switch (this) {
        case KP0:
          return 2;
        case KP1:
          return 1;
        case KP2:
          return 4;
        case KP3:
          return 3;
        case KP4:
          return 5;
        default:
          return 0;
      }
    }

    /** Deserialize from integer */
    public static Height fromInt(int value) {
      switch (value) {
        case 0:
          return KP0;
        case 1:
          return KP1;
        case 2:
          return KP0;
        case 3:
          return KP3;
        case 4:
          return KP2;
        case 5:
          return KP4;
        default:
          return Height.KP0;
      }
    }

    /**
     * @return increments coral state
     */
    public Height increment() {
      int new_ordinal = this.ordinal() + 1;
      if (new_ordinal >= Height.values().length) {
        return this;
      }
      return Height.values()[new_ordinal];
    }

    /**
     * @return decrements coral state
     */
    public Height decrement() {
      int new_ordinal = this.ordinal() - 1;
      if (new_ordinal < 0) {
        return this;
      }
      return Height.values()[new_ordinal];
    }

    public static Height currentState = Height.KP1;

    /** Get currently tracked state. */
    public static Height getCurrentState() {
      return currentState;
    }

    /** Advance state forward by one. */
    public static void incrementState() {
      currentState = currentState.increment();
    }

    /** Advance state backwards by one. */
    public static void decrementState() {
      currentState = currentState.decrement();
    }
  }
}
