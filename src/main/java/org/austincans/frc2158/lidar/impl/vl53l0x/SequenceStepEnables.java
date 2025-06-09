// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.vl53l0x;

class SequenceStepEnables {
  final boolean tcc, msrc, dss, preRange, finalRange;

  public SequenceStepEnables(
      boolean tcc, boolean msrc, boolean dss, boolean preRange, boolean finalRange) {
    this.tcc = tcc;
    this.msrc = msrc;
    this.dss = dss;
    this.preRange = preRange;
    this.finalRange = finalRange;
  }
}
