// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/** Utility class for logging code execution times. */
public class LoggedTracer {
  private LoggedTracer() {}

  private static double startTime = -1.0;

  /** Reset the clock. */
  public static void reset() {
    startTime = Timer.getFPGATimestamp();
  }

  /** Save the time elapsed since the last reset or record. */
  public static void record(String epochName) {
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput("LoggedTracer/" + epochName + "MS", (now - startTime) * 1000.0);
    startTime = now;
  }
}
