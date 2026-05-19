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

package org.austincans.lib.sensors.lidar.impl.vl53l0x;

class SequenceStepTimeouts {
  final int preRangeVcselPeriodPclks, finalRangeVcselPeriodPclks;
  final int msrcDssTccMclks, preRangeMclks, finalRangeMclks;
  final long msrcDssTccMicrosec, preRangeMicrosec, finalRangeMicrosec;

  public SequenceStepTimeouts(
      int preRangeVcselPeriodPclks,
      int finalRangeVcselPeriodPclks,
      int msrcDssTccMclks,
      int preRangeMclks,
      int finalRangeMclks,
      long msrcDssTccMicrosec,
      long preRangeMicrosec,
      long finalRangeMicrosec) {
    this.preRangeVcselPeriodPclks = preRangeVcselPeriodPclks;
    this.finalRangeVcselPeriodPclks = finalRangeVcselPeriodPclks;
    this.msrcDssTccMclks = msrcDssTccMclks;
    this.preRangeMclks = preRangeMclks;
    this.finalRangeMclks = finalRangeMclks;
    this.msrcDssTccMicrosec = msrcDssTccMicrosec;
    this.preRangeMicrosec = preRangeMicrosec;
    this.finalRangeMicrosec = finalRangeMicrosec;
  }
}
