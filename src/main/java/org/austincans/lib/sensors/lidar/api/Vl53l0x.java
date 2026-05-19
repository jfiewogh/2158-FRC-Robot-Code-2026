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

package org.austincans.lib.sensors.lidar.api;

import java.util.Optional;
import java.util.concurrent.TimeUnit;

import org.austincans.lib.sensors.lidar.arcompat.PololuI2c;
import org.austincans.lib.sensors.lidar.impl.vl53l0x.Vl53l0xI2c;

/**
 * API for interacting with a VL53L0X range-finder.
 *
 * <p>The suggested implementation to use is {@link Vl53l0xI2c}.
 */
public interface Vl53l0x extends AutoCloseable {

  enum VcselPeriodType {
    VCSEL_PERIOD_PRE_RANGE,
    VCSEL_PERIOD_FINAL_RANGE
  }

  byte DEFAULT_ADDRESS = 0x29;

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

  void setMeasurementTimingBudget(long budgetMicrosec);

  long getMeasurementTimingBudget();

  void setVcselPulsePeriod(VcselPeriodType type, short periodPclks);

  short getVcselPulsePeriod(VcselPeriodType type);

  void startContinuous(long periodMilli);

  void stopContinuous();

  boolean dataReady();

  int readRangeContinuousMillimeters();

  int readRangeSingleMillimeters();

  /** Sets the timeout for a response from the VL53L0X unit. */
  void setTimeout(long timeout, TimeUnit unit);

  /** Gets the current timeout, in the requested unit. */
  long getTimeout(TimeUnit unit);

  boolean timeoutOccurred();
}
