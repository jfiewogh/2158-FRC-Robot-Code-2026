// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.tca9548a;

import java.util.Optional;
import java.util.concurrent.TimeUnit;
import org.austincans.frc2158.lidar.api.Error;
import org.austincans.frc2158.lidar.api.Tca9548a;
import org.austincans.frc2158.lidar.api.Wire;
import org.austincans.frc2158.lidar.arcompat.PololuI2c;
import org.austincans.frc2158.lidar.arcompat.Port;

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
