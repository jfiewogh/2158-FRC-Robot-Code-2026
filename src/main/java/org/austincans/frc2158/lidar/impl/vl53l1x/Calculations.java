// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.vl53l1x;

class Calculations {

  static int encodeTimeout(int timeoutMclks) {
    if (timeoutMclks <= 0) {
      return 0;
    }

    long lsByte = timeoutMclks - 1;
    int msByte = 0;

    while ((lsByte & 0xFFFFFF00) > 0) {
      lsByte >>= 1;
      msByte++;
    }

    return (int) ((msByte << 8) | (lsByte & 0xFF));
  }

  static int decodeTimeout(int regVal) {
    return ((regVal & 0xFF) << (regVal >> 8)) + 1;
  }

  static int timeoutMclksToMicroseconds(int timeoutMclks, int macroPeriodMicrosec) {
    long tmp = ((long) timeoutMclks) * macroPeriodMicrosec;
    return (int) ((tmp + 0x800L) >> 12);
  }

  static int timeoutMicrosecondsToMclks(int timeoutMicrosec, int macroPeriodMicrosec) {
    return ((timeoutMicrosec << 12) + (macroPeriodMicrosec >> 1)) / macroPeriodMicrosec;
  }

  // "Calculate macro period in microseconds (12.12 format) with given VCSEL period"
  // TBH I have no idea what this does.
  static int calcMacroPeriod(int fastOscFreq, short vcselPeriod) {
    int pllPeriodMicrosec = (1 << 30) / fastOscFreq;
    int vcselPeriodPclks = (vcselPeriod + 1) << 1;
    int macroPeriodMicrosec = 2304 * pllPeriodMicrosec;
    macroPeriodMicrosec >>= 6;
    macroPeriodMicrosec *= vcselPeriodPclks;
    macroPeriodMicrosec >>= 6;
    return macroPeriodMicrosec;
  }
}
