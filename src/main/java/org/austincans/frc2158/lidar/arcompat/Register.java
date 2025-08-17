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
// Updated on Sunday August 17, 2025 at 08:59:14 CDT

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
