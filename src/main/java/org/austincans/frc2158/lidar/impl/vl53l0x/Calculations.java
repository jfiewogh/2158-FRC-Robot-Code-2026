// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.vl53l0x;

class Calculations {
  static short decodeVcselPeriod(short value) {
    return (short) ((value + 1) << 1);
  }

  static short encodeVcselPeriod(short value) {
    return (short) ((value >> 1) + 1);
  }

  static int calcMacroPeriod(int vcselPeriodPclks) {
    return ((2304 * vcselPeriodPclks * 1655) + 500) / 1000;
  }
}
