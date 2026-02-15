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
//
// Updated on Sunday February 15, 2026 at 11:15:12 CST

package org.ironmaple.simulation.debugging;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Simple file-based debug logger for 3D simulation debugging. Writes to /tmp/maple-sim-debug.log
 */
public class SimDebugLogger {
  private static final String LOG_FILE = "/tmp/maple-sim-debug.log";
  private static PrintWriter writer;
  private static final long startTimeMs;
  private static final java.util.concurrent.atomic.AtomicInteger tickCount =
      new java.util.concurrent.atomic.AtomicInteger(0);
  private static volatile boolean enabled = true;

  static {
    startTimeMs = System.currentTimeMillis();
    try {
      writer = new PrintWriter(new FileWriter(LOG_FILE, false)); // Overwrite on start
      writer.println("=== Maple-Sim 3D Debug Log ===");
      writer.println("Started: " + new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(new Date()));
      writer.println("Format: [time_ms] [tick] [category] message");
      writer.println("=====================================");
      writer.flush();
    } catch (IOException e) {
      System.err.println("Failed to create debug log: " + e.getMessage());
      enabled = false;
    }
  }

  public static void setEnabled(boolean enable) {
    enabled = enable;
  }

  public static void incrementTick() {
    tickCount.incrementAndGet();
  }

  public static synchronized void log(String category, String message) {
    if (!enabled || writer == null) return;
    long elapsed = System.currentTimeMillis() - startTimeMs;
    writer.printf("[%6d] [%5d] [%-12s] %s%n", elapsed, tickCount.get(), category, message);
    writer.flush();
  }

  public static void logPhysics(String message) {
    log("PHYSICS", message);
  }

  public static void logPerformance(String message) {
    log("PERF", message);
  }

  public static void logGyro(String message) {
    log("GYRO", message);
  }

  public static void logHeading(String message) {
    log("HEADING", message);
  }

  public static void logForce(String message) {
    log("FORCE", message);
  }

  public static void logVelocity(String message) {
    log("VELOCITY", message);
  }

  public static void logPose(String message) {
    log("POSE", message);
  }

  public static synchronized void close() {
    if (writer != null) {
      writer.close();
    }
  }
}
