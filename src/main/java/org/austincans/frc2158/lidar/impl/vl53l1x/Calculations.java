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
// Updated on Thursday November 13, 2025 at 17:43:29 CST

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
