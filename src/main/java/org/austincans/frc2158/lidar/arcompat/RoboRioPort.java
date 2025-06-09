// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.arcompat;

/** Ports available on the RoboRIO board. */
public enum RoboRioPort implements Port {
  /** The I2C port on the RoboRIO's board. */
  ONBOARD(0),
  /** The I2C port on the MXP output. */
  MXP(1);

  private final int port;

  RoboRioPort(int port) {
    this.port = port;
  }

  @Override
  public int value() {
    return port;
  }
}
