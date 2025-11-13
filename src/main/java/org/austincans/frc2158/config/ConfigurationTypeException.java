// Copyright (c) 2025 the ausTIN CANs
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
//
// Updated on Thursday November 13, 2025 at 17:43:29 CST

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
