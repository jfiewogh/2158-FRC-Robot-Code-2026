// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.vl53l0x;

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
