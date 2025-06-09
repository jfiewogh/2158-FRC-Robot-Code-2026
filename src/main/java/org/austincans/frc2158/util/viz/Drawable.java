// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util.viz;

import org.austincans.frc2158.util.ConfigUtil;

/** Something that can be drawn to AdvantageScope. */
public interface Drawable {

  /** Draw to AdvantageScope unconditionally. */
  public void drawImpl();

  /** Draw to AdvantageScope if drawing is enabled. */
  public default void draw() {
    if (ConfigUtil.getConfigBoolean("SHOULD_DRAW", false)) {
      drawImpl();
    }
  }
}
