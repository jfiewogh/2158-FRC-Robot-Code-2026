// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.util;

import java.util.concurrent.TimeUnit;

public class SleepEasy {

  /**
   * Wrapper for {@link TimeUnit#sleep(long)} that handles the InterruptedException by interrupting
   * the current thread again, and throwing an exception.
   */
  public static void forUnit(long time, TimeUnit unit) {
    try {
      unit.sleep(time);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new RuntimeException(e);
    }
  }
}
