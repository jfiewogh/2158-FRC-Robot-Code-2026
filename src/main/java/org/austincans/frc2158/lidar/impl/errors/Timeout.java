// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.errors;

import com.google.auto.value.AutoValue;
import org.austincans.frc2158.lidar.impl.ErrorBase;

public class Timeout extends ErrorBase<Timeout.Data> {

  public static Timeout waitingFor(String waitingFor) {
    return new Timeout(new AutoValue_Timeout_Data(waitingFor));
  }

  @AutoValue
  public abstract static class Data {

    public abstract String waitingFor();

    @Override
    public final String toString() {
      return "Timed out waiting for " + waitingFor();
    }
  }

  private Timeout(Data data) {
    super("timeout", data);
  }
}
