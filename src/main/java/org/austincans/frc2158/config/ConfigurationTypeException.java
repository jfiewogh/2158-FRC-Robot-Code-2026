// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.config;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * An exception that is raised when a configuration contains some value that cannot be converted to
 * the requested type.
 *
 * @since 2.0.0
 */
public final class ConfigurationTypeException extends ConfigurationException {

  ConfigurationTypeException(
      @NotNull String path, @NotNull Class<?> requestedType, @Nullable Object value) {
    super(prepareMessage(path, requestedType, value));
  }

  private static @NotNull String prepareMessage(
      @NotNull String path, @NotNull Class<?> requestedType, @Nullable Object value) {
    StringBuilder builder = new StringBuilder();

    if (value == null) {
      builder.append("value `null`");
    } else if (value instanceof ConfigurationSection) {
      builder
          .append(ConfigurationSection.class.getPackageName())
          .append(".")
          .append(ConfigurationSection.class.getSimpleName());
    } else {
      builder.append("value `").append(value).append("` of type ").append(value.getClass());
    }

    builder
        .append(" found in path `")
        .append(path)
        .append("` cannot be converted to ")
        .append(requestedType);

    return builder.toString();
  }
}
