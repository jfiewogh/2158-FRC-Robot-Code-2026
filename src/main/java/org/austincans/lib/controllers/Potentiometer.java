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

package org.austincans.frc2026.controllers;

import org.austincans.lib.math.CANMath;

import edu.wpi.first.wpilibj.AnalogInput;

public class Potentiometer extends AnalogInput {
  private double minAngle = 0, maxAngle = 180; // Classic 0 to 180 degrees
  private double minPotValue = 0, maxPotValue = 4096; // Classic 4KOhms potentiometer, e.g. encoder
  private double absoluteReset = 0;
  private boolean potDirection = false;

  /**
   * Set Potentiometer port
   *
   * @param port
   */
  public Potentiometer(final int port) {
    super(port);
  }

  /**
   * Get the minimum angle for this potentiometer
   *
   * @return the minimum angle in degrees
   */
  public double getMinAngle() {
    return minAngle;
  }

  /**
   * Set the minimum angle for this potentiometer
   *
   * @param minAngle the minimum angle in degrees
   */
  public void setMinAngle(double minAngle) {
    this.minAngle = minAngle;
  }

  /**
   * Get the maximum angle for this potentiometer
   *
   * @return the maximum angle in degrees
   */
  public double getMaxAngle() {
    return maxAngle;
  }

  /**
   * Set the maximum angle for this potentiometer
   *
   * @param maxAngle the maximum angle in degrees
   */
  public void setMaxAngle(double maxAngle) {
    this.maxAngle = maxAngle;
  }

  /**
   * Get the minimum value for this potentiometer
   *
   * @return the minimum value in Ohms
   */
  public double getMinPotValue() {
    return minPotValue;
  }

  /**
   * Set the minimum value for this potentiometer
   *
   * @param minPotValue the minimum value in Ohms
   */
  public void setMinPotValue(double minPotValue) {
    this.minPotValue = minPotValue;
  }

  /**
   * Get the maximum value for this potentiometer
   *
   * @return the maximum value in Ohms
   */
  public double getMaxPotValue() {
    return maxPotValue;
  }

  /**
   * Set the maximum value for this potentiometer
   *
   * @param maxPotValue the maximum value in Ohms
   */
  public void setMaxPotValue(double maxPotValue) {
    this.maxPotValue = maxPotValue;
  }

  /** Reset the angle */
  public void resetAngle() {
    absoluteReset = getAbsoluteAngle();
    potDirection = absoluteReset > 0; // false == less, true == more
  }

  /**
   * Get the angle
   *
   * @return the angle in degrees
   */
  public double getAngle() {
    final double currentAngle = getAbsoluteAngle();
    return potDirection ? currentAngle - absoluteReset : currentAngle + absoluteReset;
  }

  /**
   * Get the absolute angle
   *
   * @return the angle in degrees
   */
  public double getAbsoluteAngle() {
    return -CANMath.map(getAverageVoltage(), minPotValue, maxPotValue, minAngle, maxAngle);
  }
}
