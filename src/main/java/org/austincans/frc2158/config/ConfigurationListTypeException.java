// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.config;

import java.util.List;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * An exception that is raised when a list inside a configuration contains some element that cannot
 * be converted to the requested list type.
 *
 * @since 2.0.0
 */
public final class ConfigurationListTypeException extends ConfigurationException {

  ConfigurationListTypeException(
      @NotNull String path,
      @NotNull Class<?> requestedType,
      @NotNull List<?> list,
      @Nullable Object value) {
    super(prepareMessage(path, requestedType, list, value));
  }

  private static @NotNull String prepareMessage(
      @NotNull String path,
      @NotNull Class<?> requestedType,
      @NotNull List<?> list,
      @Nullable Object value) {
    StringBuilder builder = new StringBuilder();

    builder
        .append("list `")
        .append(list)
        .append("` found in path `")
        .append(path)
        .append("` could not be converted to ")
        .append(List.class)
        .append("<")
        .append(requestedType)
        .append("> because it contains ");

    if (value == null) {
      builder.append("null element(s)");
    } else {
      builder
          .append("value `")
          .append(value)
          .append("` of type ")
          .append(value.getClass())
          .append(" that cannot be converted to ")
          .append(requestedType);
    }

    return builder.toString();
  }
}
