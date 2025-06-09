// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.vl53l0x.errors;

import com.google.auto.value.AutoValue;
import org.austincans.frc2158.lidar.impl.ErrorBase;

public class NoSpadInfo extends ErrorBase<NoSpadInfo.Data> {

  public static NoSpadInfo getInstance() {
    return new NoSpadInfo(new AutoValue_NoSpadInfo_Data());
  }

  @AutoValue
  public abstract static class Data {

    @Override
    public final String toString() {
      return "No spad info";
    }
  }

  private NoSpadInfo(Data data) {
    super("no.spad.info", data);
  }
}
