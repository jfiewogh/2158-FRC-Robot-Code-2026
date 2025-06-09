// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.util;

/** Low-quality variant of Guava's Preconditions. */
public class Preconditions {

  public static void checkArgument(boolean condition, String message) {
    if (!condition) {
      throw new IllegalArgumentException(message);
    }
  }

  public static void checkArgument(boolean condition, String message, Object... parameters) {
    if (!condition) {
      throw new IllegalArgumentException(String.format(message, parameters));
    }
  }

  public static void checkState(boolean condition, String message) {
    if (!condition) {
      throw new IllegalStateException(message);
    }
  }

  public static void checkState(boolean condition, String message, Object... parameters) {
    if (!condition) {
      throw new IllegalStateException(String.format(message, parameters));
    }
  }
}
