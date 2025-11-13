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

public class IntRange {
  private int min;
  private int max;

  /**
   * Swap min and max based on value if min > max than max value is min and min value is max
   *
   * @param min
   * @param max
   */
  public IntRange(int min, int max) {
    this.min = min;
    this.max = max;

    if (min > max) {
      this.min = max;
      this.max = min;
    }
  }

  /**
   * Get the minimum value
   *
   * @return the minimum value as integer
   */
  public int getMin() {
    return this.min;
  }

  /**
   * Get the maximum value
   *
   * @return the maximum value as integer
   */
  public int getMax() {
    return this.max;
  }

  /**
   * Check if parameter is withing range defined by min and max value
   *
   * @param n
   * @return true if in range, false otherwise
   */
  public boolean inRange(int n) {
    return n >= this.min && n <= this.max;
  }

  /**
   * Return the greater of parameter and max value If parameter is greater than max, return max
   *
   * @param n
   * @return the clamp value as integer
   */
  public int clamp(int n) {
    return Math.max(this.min, Math.min(this.max, n));
  }

  /**
   * Return the clamp value based on min and max value return min if less than min, max if greater
   * than max and parameter value if otherwise in range
   *
   * @param n
   * @return the clamp value as integer
   */
  public double clamp(double n) {
    return Math.max(this.min, Math.min(this.max, n));
  }

  /**
   * Return the greater of the 2 float value, clamp to max value
   *
   * @param n
   * @return the clamp value as float
   */
  public float clamp(float n) {
    return Math.max(this.min, Math.min(this.max, n));
  }

  /**
   * Return the greater of the 2 long value, clamp to max value
   *
   * @param n
   * @return the clamp value as long
   */
  public long clamp(long n) {
    return Math.max(this.min, Math.min(this.max, n));
  }
}
