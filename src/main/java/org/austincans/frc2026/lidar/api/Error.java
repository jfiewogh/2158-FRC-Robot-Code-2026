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

package org.austincans.frc2026.lidar.api;

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
