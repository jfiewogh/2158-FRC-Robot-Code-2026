// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.api;

import java.util.Optional;
import java.util.function.Consumer;

/**
 * Represents some kind of error.
 *
 * <p>Each error has a code and some data. The string representation is also suitable for easy
 * reporting.
 *
 * <p>Many API methods return {@code Optional<Error<?>>}. This should be handled by {@link
 * Optional#ifPresent(Consumer)} in most cases, where you throw an exception better detailing the
 * error for your use case.
 */
public interface Error<T> {

  String code();

  T data();

  /**
   * Provides a reportable representation of the error.
   *
   * <p>Typically this will look like {@code [$code] $message}.
   */
  @Override
  String toString();
}
