// Copyright (c) 2025 the ausTIN CANs
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
//
// Updated on Wednesday November 05, 2025 at 18:44:58 CST

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
