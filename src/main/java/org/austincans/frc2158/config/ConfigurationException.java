// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.config;

import org.jetbrains.annotations.NotNull;

/**
 * An exception that is raised when specific situations emerge when interacting with configurations
 * in bad ways.
 *
 * @since 2.0.0
 */
public class ConfigurationException extends RuntimeException {

  ConfigurationException(@NotNull String message) {
    super(message);
  }
}
