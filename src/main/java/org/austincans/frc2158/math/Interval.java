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
// Updated on Thursday November 13, 2025 at 14:16:40 CST

package org.austincans.frc2158.math;

/**
 * A one-dimensional range. This class is used to represent a range of values, such as the range of
 * a sensor or the range of a motor. It provides methods to check if two ranges overlap, get the
 * length of the overlap, and check if one range contains another.
 *
 * @author Austin Cans Robotics Team 2158
 */
public class Interval {

  private double min;
  private double max;

  /** A one-dimensional range. */
  public Interval(double min, double max) {
    this.min = min;
    this.max = max;
  }

  /** Get lower extent of the range. */
  public double getMin() {
    return min;
  }

  /** Set lower extent of the range. */
  public void setMin(double min) {
    this.min = min;
  }

  /** Get upper extent of the range. */
  public double getMax() {
    return max;
  }

  /** Set upper extent of the range. */
  public void setMax(double max) {
    this.max = max;
  }

  /** Get if two intervals share some common subset. */
  public boolean overlaps(Interval other) {
    return !(this.min > other.max || other.min > this.max);
  }

  /** Get length of common subset. */
  public double getOverlap(Interval other) {
    // make sure they overlap
    if (this.overlaps(other)) {
      return Math.min(this.max, other.max) - Math.max(this.min, other.min);
    }
    return 0;
  }

  /** Get if `other` is completely contained by this range. */
  public boolean contains(Interval other) {
    return other.min > this.min && other.max < this.max;
  }
}
