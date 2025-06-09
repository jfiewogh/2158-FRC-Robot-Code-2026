// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.api;

import java.util.Optional;
import java.util.concurrent.TimeUnit;
import org.austincans.frc2158.lidar.arcompat.PololuI2c;
import org.austincans.frc2158.lidar.impl.tca9548a.Tca9548aI2c;

/**
 * API for interacting with a TCA9548A I2C Multiplexer.
 *
 * <p>The suggested implementation to use is {@link Tca9548aI2c}.
 */
public interface Tca9548a extends AutoCloseable {

  byte DEFAULT_ADDRESS = 0x70;

  /**
   * @return the current address
   */
  byte getAddress();

  /**
   * Sets the current address to communicate with.
   *
   * <p>This will change the address of the VL53L0X to match. If you only want to change the address
   * of this object, use {@link PololuI2c#setAddress(byte)}.
   *
   * @param address the new address
   */
  void setAddress(byte address);

  /**
   * @return the underlying I2C communication helper
   */
  PololuI2c getI2c();

  void TcaSelect(int i2cAddress);

  /**
   * Initialize. Check if we're talking to the right module, etc.
   *
   * @return {@link Optional#empty()} if the module is initialized
   */
  Optional<Error<?>> initialize();

  @Override
  void close();

  void setSignalRateLimit(float limitMpcs);

  float getSignalRateLimit();

  boolean dataReady();

  /** Sets the timeout for a response from the VL53L0X unit. */
  void setTimeout(long timeout, TimeUnit unit);

  /** Gets the current timeout, in the requested unit. */
  long getTimeout(TimeUnit unit);

  boolean timeoutOccurred();
}
