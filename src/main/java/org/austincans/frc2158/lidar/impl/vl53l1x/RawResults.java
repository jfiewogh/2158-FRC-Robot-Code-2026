// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.vl53l1x;

import com.google.auto.value.AutoValue;

@AutoValue
abstract class RawResults {

  static Builder builder() {
    return new AutoValue_RawResults.Builder();
  }

  @AutoValue.Builder
  interface Builder {

    Builder rangeStatus(short rangeStatus);

    Builder streamCount(short streamCount);

    Builder dssActualEffectiveSpadsSd0(int dssActualEffectiveSpadsSd0);

    Builder ambientCountRateMcpsSd0(int ambientCountRateMcpsSd0);

    Builder finalCrosstalkCorrectRangeMmSd0(int finalCrosstalkCorrectRangeMmSd0);

    Builder peakSignalCountRateCrosstalkCorrectedMcpsSd0(
        int peakSignalCountRateCrosstalkCorrectedMcpsSd0);

    RawResults build();
  }

  RawResults() {}

  abstract short rangeStatus();

  abstract short streamCount();

  abstract int dssActualEffectiveSpadsSd0();

  abstract int ambientCountRateMcpsSd0();

  abstract int finalCrosstalkCorrectRangeMmSd0();

  abstract int peakSignalCountRateCrosstalkCorrectedMcpsSd0();
}
