// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util;

import edu.wpi.first.wpilibj.Threads;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

/** Dummy log receiver used to adjust the thread priority of the log receiver thread. */
public class DummyLogReceiver implements LogDataReceiver {
  @Override
  public void start() {
    // DO NOT COPY UNLESS YOU UNDERSTAND THE CONSEQUENCES
    // https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template#real-time-thread-priority
    Threads.setCurrentThreadPriority(true, 1);
  }

  @Override
  public void putTable(LogTable table) throws InterruptedException {}
}
