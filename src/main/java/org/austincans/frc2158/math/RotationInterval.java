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
// Updated on Wednesday November 05, 2025 at 18:44:58 CST

package org.austincans.frc2158.math;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A one-dimensional range with wrapping on [-pi,pi]. This is used to represent the range of angles
 * that a robot can turn to face a target. It is used in the PathPlanner library to represent the
 * range of angles that a robot can turn to face a target.
 *
 * @author Austin Cans Robotics Team 2158
 */
public class RotationInterval {

  private double min = 0.0;
  private double max = 0.0;

  /** A one-dimensional range with wrapping on [-pi,pi]. */
  public RotationInterval(Rotation2d min, Rotation2d max) {
    setMax(max);
    setMin(min);
  }

  private RotationInterval() {}

  /** A one-dimensional range with wrapping on [-pi,pi]. */
  public static RotationInterval acute(Rotation2d a, Rotation2d b) {
    RotationInterval interval = new RotationInterval(a, b);
    if (interval.range() > Math.PI) {
      return interval.complement();
    } else {
      return interval;
    }
  }

  /** Get the interval opposite this. */
  public RotationInterval complement() {
    RotationInterval compl = new RotationInterval();
    compl.max = this.min;
    compl.min = this.max;
    return compl;
  }

  /** Get distance from min to max. */
  public double range() {
    if (min > max) {
      return (2 * Math.PI - min) + max;
    } else {
      return max - min;
    }
  }

  /** Get lower extent of the range. */
  public Rotation2d getMin() {
    return Rotation2d.fromRadians(min);
  }

  /** Set lower extent of the range. */
  public void setMin(Rotation2d min) {
    this.min = Math.atan2(min.getSin(), min.getCos());
  }

  /** Get upper extent of the range. */
  public Rotation2d getMax() {
    return Rotation2d.fromRadians(max);
  }

  /** Set upper extent of the range. */
  public void setMax(Rotation2d max) {
    this.max = Math.atan2(max.getSin(), max.getCos());
  }

  /** Get if two intervals share some angles. */
  public boolean overlaps(RotationInterval other) {
    return getOverlap(other) > 0;
  }

  /** Get length of common subset. */
  public double getOverlap(RotationInterval other) {
    if (min > max) {
      if (other.min > other.max) {
        // Both cross 0
        return getOverlap(this.min, Math.PI, other.min, Math.PI)
            + getOverlap(this.min, Math.PI, -Math.PI, other.max)
            + getOverlap(-Math.PI, this.max, other.min, Math.PI)
            + getOverlap(-Math.PI, this.max, -Math.PI, other.max);
      } else {
        // Only this crosses 0
        return getOverlap(this.min, Math.PI, other.min, other.max)
            + getOverlap(-Math.PI, this.max, other.min, other.max);
      }
    } else {
      if (other.min > other.max) {
        // Only other crosses 0
        return getOverlap(this.min, this.max, other.min, Math.PI)
            + getOverlap(this.min, this.max, -Math.PI, other.max);
      } else {
        // Neither cross 0
        return getOverlap(this.min, this.max, other.min, other.max);
      }
    }
  }

  private static double getOverlap(double al, double ah, double bl, double bh) {
    if (!(bl > ah || al > bh)) {
      return Math.min(ah, bh) - Math.max(al, bl);
    } else {
      return 0;
    }
  }
}
