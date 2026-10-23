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

package org.austincans.lib.util.math.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** A class to represent a 2d cartesian coordinate. */
public class Cartesian2d {
  private double r;
  private Rotation2d theta;

  /**
   * Creates a new Cartesian2d
   *
   * @param r The radius
   * @param theta The angle
   */
  public Cartesian2d(double r, Rotation2d theta) {
    assert r >= 0 : "Radius must be positive!";

    if (theta == null) {
      throw new IllegalArgumentException("Angle cannot be null!");
    }

    // Run incase assert is disabled
    if (r < 0) {
      throw new IllegalArgumentException("Radius must be positive!");
    }

    this.r = r;
    this.theta = theta;
  }

  /**
   * Returns the radius
   *
   * @return radius
   */
  public double getR() {
    return r;
  }

  /**
   * Returns the angle
   *
   * @return angle (Rotation2d)
   */
  public Rotation2d getTheta() {
    return theta;
  }

  /**
   * Returns the angle in degrees
   *
   * @param other
   * @return
   */
  public Cartesian2d add(Cartesian2d other) {
    return new Cartesian2d(this.r + other.r, this.theta.plus(other.theta));
  }

  /**
   * Converts class into a Translation2d
   *
   * @return Translation2d
   */
  public Translation2d toTranslation2d() {
    return new Translation2d(r * Math.cos(theta.getRadians()), r * Math.sin(theta.getRadians()));
  }

  /**
   * Converts a Translation2d into a Cartesian2d
   *
   * @param translation2d The Translation2d to convert
   * @return Cartesian2d
   */
  public static Cartesian2d fromTranslation2d(Translation2d translation2d) {
    return new Cartesian2d(
        Math.hypot(translation2d.getX(), translation2d.getY()),
        new Rotation2d(Math.atan2(translation2d.getY(), translation2d.getX())));
  }
}
