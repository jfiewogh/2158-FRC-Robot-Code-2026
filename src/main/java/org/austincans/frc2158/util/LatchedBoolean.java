// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util;

/**
 * An iterative boolean latch.
 *
 * <p>Returns true once if and only if the value of newValue changes from false to true.
 *
 * @author ustin Cans Robotics Team 2158
 */
public class LatchedBoolean {
  private boolean mLast = false;

  public boolean update(boolean newValue) {
    boolean ret = false;
    if (newValue && !mLast) {
      ret = true;
    }
    mLast = newValue;
    return ret;
  }
}
