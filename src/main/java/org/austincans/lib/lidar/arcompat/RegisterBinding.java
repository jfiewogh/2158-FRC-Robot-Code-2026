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

package org.austincans.lib.lidar.arcompat;

class RegisterBinding implements Register.Bound {

  private final short reg;
  private final PololuI2c i2c;

  RegisterBinding(short reg, PololuI2c i2c) {
    this.reg = reg;
    this.i2c = i2c;
  }

  @Override
  public boolean write(short value) {
    return i2c.writeReg(reg, value);
  }

  @Override
  public boolean write16Bit(int value) {
    return i2c.writeReg16Bit(reg, value);
  }

  @Override
  public boolean write32Bit(long value) {
    return i2c.writeReg32Bit(reg, value);
  }

  @Override
  public boolean writeMulti(byte[] value) {
    return i2c.writeRegMulti(reg, value);
  }

  @Override
  public short read() {
    return i2c.readReg(reg);
  }

  @Override
  public int read16Bit() {
    return i2c.readReg16Bit(reg);
  }

  @Override
  public long read32Bit() {
    return i2c.readReg32Bit(reg);
  }

  @Override
  public void readMulti(byte[] result) {
    i2c.readRegMulti(reg, result);
  }
}
