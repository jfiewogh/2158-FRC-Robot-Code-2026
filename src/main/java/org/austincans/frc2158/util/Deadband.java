// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util;

/*
 * @author Austin Cans Robotics Team 2158
 */
public class Deadband {
  public static double applyLowHigh(
      double input, double lowPass, double highPass, double defaultValue) {
    if (inDeadbandLowHigh(input, lowPass, highPass)) {
      return defaultValue;
    }
    return input;
  }

  public static boolean inDeadband(double input, double window) {
    return inDeadbandLowHigh(input, -window, window);
  }

  public static boolean inDeadbandLowHigh(double input, double lowPass, double highPass) {
    return input < highPass && input > lowPass;
  }

  public static double apply(double input, double window) {
    if (inDeadband(input, window)) {
      return 0;
    }
    return input;
  }
}
