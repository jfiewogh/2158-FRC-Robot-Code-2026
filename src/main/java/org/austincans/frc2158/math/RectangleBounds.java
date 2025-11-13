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
// Updated on Wednesday November 12, 2025 at 18:18:23 CST

package org.austincans.frc2158.math;

import edu.wpi.first.math.geometry.Translation2d;

public class RectangleBounds {
  private double x;
  private double y;
  private double width;
  private double height;

  /**
   * Constructor
   *
   * @param x
   * @param y
   * @param width
   * @param height
   */
  public RectangleBounds(double x, double y, double width, double height) {
    this.x = x;
    this.y = y;
    this.width = width;
    this.height = height;
  }

  /**
   * Constructor
   *
   * @param position
   * @param width
   * @param height
   */
  public RectangleBounds(Translation2d position, double width, double height) {
    this.x = position.getX();
    this.y = position.getY();
    this.width = width;
    this.height = height;
  }

  /**
   * Get position
   *
   * @return the position as a Translation2d
   */
  public Translation2d getPosition() {
    return new Translation2d(x, y);
  }

  /**
   * Get the center of x
   *
   * @return the value as a double
   */
  public double getCenterX() {
    return x + width / 2;
  }

  /**
   * Get the center of y
   *
   * @return the value as double
   */
  public double getCenterY() {
    return y + height / 2;
  }

  /**
   * get the value of x
   *
   * @return the value as a double
   */
  public double getX() {
    return x;
  }

  /**
   * get the value of y
   *
   * @return the value as a double
   */
  public double getY() {
    return y;
  }

  /**
   * get the rectangle width
   *
   * @return the value as a double
   */
  public double getWidth() {
    return width;
  }

  /**
   * get the rectangle height
   *
   * @return the value as a double
   */
  public double getHeight() {
    return height;
  }
}
