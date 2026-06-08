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

package org.austincans.lib.lidar.api;

import java.util.Optional;
import java.util.concurrent.TimeUnit;

import org.austincans.lib.lidar.arcompat.PololuI2c;
import org.austincans.lib.lidar.impl.vl6180x.Vl6180xI2c;

/**
 * API for interacting with a VL6180X range-finder.
 *
 * <p>The suggested implementation to use is {@link Vl6180xI2c}.
 */
public interface Vl6180x extends AutoCloseable {

  /** Scaling values, for 1x, 2x, and 3x. */
  enum Scaling {
    ONE_TIMES,
    TWO_TIMES,
    THREE_TIMES;

    public final int amount() {
      return ordinal() + 1;
    }
  }

  byte DEFAULT_ADDRESS = 0x29;

  /**
   * @return the current address
   */
  byte getAddress();

  /**
   * Sets the current address to communicate with.
   *
   * <p>This will change the address of the VL6180X to match. If you only want to change the address
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

  /** Configure sensible defaults. */
  void configureDefault();

  @Override
  void close();

  /**
   * Sets range scaling factor. The sensor uses 1x scaling by default, giving range measurements in
   * units of mm. Increasing the scaling to 2x or 3x makes it give raw values in units of 2 mm or 3
   * mm instead. In other words, a bigger scaling factor increases the sensor's potential maximum
   * range but reduces its resolution.
   */
  void setScaling(Scaling scaling);

  Scaling getScaling();

  short readRangeSingle();

  default int readRangeSingleMillimeters() {
    return getScaling().amount() * readRangeSingle();
  }

  int readAmbientSingle();

  void startRangeContinuous(int period);

  void startAmbientContinuous(int period);

  void startInterleavedContinuous(int period);

  void stopContinuous();

  boolean dataReadyRange();

  /**
   * Read range data, if there is {@linkplain #dataReadyRange() data ready}.
   *
   * @throws IllegalStateException if there is no data ready
   */
  short readRangeContinuous();

  /**
   * Read range data, if there is {@linkplain #dataReadyRange() data ready}.
   *
   * @throws IllegalStateException if there is no data ready
   */
  default int readRangeContinuousMillimeters() {
    return getScaling().amount() * readRangeContinuous();
  }

  boolean dataReadyAmbient();

  /**
   * Read ambient data, if there is {@linkplain #dataReadyAmbient() data ready}.
   *
   * @throws IllegalStateException if there is no data ready
   */
  int readAmbientContinuous();

  /** Sets the timeout for a response from the VL6180X unit. */
  void setTimeout(long timeout, TimeUnit unit);

  /** Gets the current timeout, in the requested unit. */
  long getTimeout(TimeUnit unit);

  boolean timeoutOccurred();
}
