// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.vl53l0x;

class SpadInfo {
  final short count;
  final boolean isAperture;

  SpadInfo(short count, boolean isAperture) {
    this.count = count;
    this.isAperture = isAperture;
  }
}
