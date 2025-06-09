// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.vl6180x;

public class Calculations {
  public static short constrain(short value, short min, short max) {
    if (value < min) {
      return min;
    } else if (value > max) {
      return max;
    }
    return value;
  }
}
