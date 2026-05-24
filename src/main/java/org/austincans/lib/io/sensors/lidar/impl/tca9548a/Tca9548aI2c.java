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

package org.austincans.lib.io.sensors.lidar.impl.tca9548a;

import java.util.Optional;
import java.util.concurrent.TimeUnit;

import org.austincans.lib.io.sensors.lidar.api.Error;
import org.austincans.lib.io.sensors.lidar.api.Tca9548a;
import org.austincans.lib.io.sensors.lidar.api.Wire;
import org.austincans.lib.io.sensors.lidar.arcompat.PololuI2c;
import org.austincans.lib.io.sensors.lidar.arcompat.Port;

public class Tca9548aI2c implements Tca9548a {

  private final PololuI2c i2c;
  short stopVariable;
  long measurementTimingBudgetMicrosec;

  public Tca9548aI2c(Port port) {
    this.i2c = PololuI2c.create(port, PololuI2c.Addressing.EIGHT_BIT);
    i2c.setAddress(Tca9548a.DEFAULT_ADDRESS);
  }

  public void TcaSelect(int i2cPort) {
    i2c.beginTransmission();
    Wire wire = i2c.getWire();
    wire.writeShort(i2cPort);
    i2c.endTransmission();
  }

  @Override
  public void close() {
    i2c.close();
  }

  @Override
  public byte getAddress() {
    return i2c.getAddress();
  }

  @Override
  public void setAddress(byte address) {
    if (address == getAddress()) {
      return;
    }
    i2c.setAddress(address);
  }

  @Override
  public Optional<Error<?>> initialize() {
    return new Tca9548aInit(this).initialize();
  }

  @Override
  public PololuI2c getI2c() {
    return i2c;
  }

  @Override
  public void setSignalRateLimit(float limitMpcs) {
    throw new UnsupportedOperationException("Unimplemented method 'setSignalRateLimit'");
  }

  @Override
  public float getSignalRateLimit() {
    throw new UnsupportedOperationException("Unimplemented method 'getSignalRateLimit'");
  }

  @Override
  public boolean dataReady() {
    throw new UnsupportedOperationException("Unimplemented method 'dataReady'");
  }

  @Override
  public void setTimeout(long timeout, TimeUnit unit) {
    throw new UnsupportedOperationException("Unimplemented method 'setTimeout'");
  }

  @Override
  public long getTimeout(TimeUnit unit) {
    throw new UnsupportedOperationException("Unimplemented method 'getTimeout'");
  }

  @Override
  public boolean timeoutOccurred() {
    throw new UnsupportedOperationException("Unimplemented method 'timeoutOccurred'");
  }
}
