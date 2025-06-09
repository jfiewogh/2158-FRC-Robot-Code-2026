// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.errors;

import com.google.auto.value.AutoValue;
import org.austincans.frc2158.lidar.impl.ErrorBase;

public class IncorrectModelId extends ErrorBase<IncorrectModelId.Data> {

  public static IncorrectModelId of(String partId, int expectedId, int actualId) {
    return new IncorrectModelId(new AutoValue_IncorrectModelId_Data(partId, expectedId, actualId));
  }

  @AutoValue
  public abstract static class Data {

    public abstract String partId();

    public abstract int expectedId();

    public abstract int actualId();

    @Override
    public final String toString() {
      return String.format(
          "%s expected to have model ID 0x%02x, got 0x%02x", partId(), expectedId(), actualId());
    }
  }

  private IncorrectModelId(Data data) {
    super("incorrect.model.id", data);
  }
}
