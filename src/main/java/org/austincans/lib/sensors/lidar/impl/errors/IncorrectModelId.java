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

package org.austincans.lib.sensors.lidar.impl.errors;

import org.austincans.lib.sensors.lidar.impl.ErrorBase;

import com.google.auto.value.AutoValue;

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
