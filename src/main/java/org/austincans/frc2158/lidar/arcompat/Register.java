// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.arcompat;

/**
 * Simplistic API for registers on the I2C target. Usually implemented by an enum for easy
 * iteration.
 */
public interface Register {

  interface Bound {
    boolean write(short value);

    boolean write16Bit(int value);

    boolean write32Bit(long value);

    boolean writeMulti(byte[] value);

    // Unsigned values, so one size too large:
    short read();

    int read16Bit();

    long read32Bit();

    void readMulti(byte[] result);
  }

  short address();

  default Bound on(PololuI2c i2c) {
    return new RegisterBinding(address(), i2c);
  }
}
