// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util;

/**
 * Deadzone Utilities
 *
 * @author Austin Cans Robotics Team 2158
 */
public class Deadzone {

  /** Make deadzone for an input axis, with proper scaling. */
  public static double applyDeadzone(double input) {
    Double stickDeadBand = ConfigUtil.getConfigDouble("STICK_DEADBAND", 0.1);
    return (Math.abs(input) < stickDeadBand ? 0 : (input - stickDeadBand) / (1.0 - stickDeadBand));
  }
}
