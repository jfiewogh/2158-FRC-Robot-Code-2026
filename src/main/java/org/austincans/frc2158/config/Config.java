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
// Updated on Thursday November 13, 2025 at 14:16:40 CST

package org.austincans.frc2158.config;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import org.jetbrains.annotations.NotNull;

/**
 * An optional annotation used on {@link ConfigurationDescription}'s enum constants to provide
 * information on how to load the associated {@link Configuration} through a {@link
 * ConfigurationManager} instance.
 *
 * @since 2.0.0
 */
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD})
public @interface Config {

  /**
   * Specifies the {@link Configuration}'s path, relative to the {@link ConfigurationManager}'s
   * directory.
   *
   * <p>If not specified, the path is automatically computed based on annotation {@link Enum#name()
   * name()}.
   *
   * @return the {@link Configuration}'s path, relative to the {@link ConfigurationManager}'s
   *     directory.
   * @since 2.0.0
   */
  @NotNull
  String path() default ConfigDefaults.PATH;

  /**
   * Specifies the {@link Configuration}'s default resource path inside jar resources.
   *
   * <p>If not specified, {@link #path()} is used instead.
   *
   * @return the {@link Configuration}'s default resource path inside jar resources.
   * @since 2.0.0
   */
  @NotNull
  String defaultResource() default ConfigDefaults.RESOURCE;

  /**
   * Specifies whether the associated {@link Configuration} should be loaded with enabled auto save.
   *
   * <p>Auto save is disabled by default.
   *
   * @return whether the associated {@link Configuration} should be loaded with enabled auto save.
   * @since 2.0.0
   */
  boolean autoSave() default ConfigDefaults.AUTO_SAVE;

  /**
   * Specifies whether the associated {@link Configuration} should be saved from the default
   * resource if the file is missing when {@link ConfigurationManager#load(Enum)
   * ConfigurationManager.load()} variants are called.
   *
   * <p>Enabled by default.
   *
   * @return whether the associated {@link Configuration} should be saved from the default resource
   *     if the file is missing when {@link ConfigurationManager#load(Enum)
   *     ConfigurationManager.load()} variants are called.
   * @since 2.0.0
   */
  boolean saveDefaultResource() default ConfigDefaults.SAVE_DEFAULT_RESOURCE;
}
