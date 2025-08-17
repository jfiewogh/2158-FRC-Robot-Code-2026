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
// Updated on Sunday August 17, 2025 at 08:59:14 CDT

package org.austincans.frc2158.config;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import org.jetbrains.annotations.NotNull;

/**
 * An object used to load a {@link Configuration} object from file according to options provided
 * through its method calls.
 *
 * @since 2.0.0
 */
public final class ConfigurationLoader {

  private final File file;

  private boolean autoSave = false;
  private boolean saveDefaultResource = false;
  private String defaultResource = null;

  private ConfigurationLoader(@NotNull File file) {
    this.file = file;
  }

  /**
   * Provides information on which file is to be loaded.
   *
   * @param file the file that is to be loaded.
   * @return a new configuration loader.
   * @since 2.0.0
   */
  public static @NotNull ConfigurationLoader fromFile(@NotNull File file) {
    return new ConfigurationLoader(file);
  }

  /**
   * Provides information on which file is to be loaded.
   *
   * @param file the string representation of the path to the file that is to be loaded.
   * @return a new configuration loader.
   * @since 2.0.0
   */
  public static @NotNull ConfigurationLoader fromFile(@NotNull String file) {
    return fromFile(new File(file));
  }

  /**
   * Provides information on which file is to be loaded.
   *
   * @param directory the directory inside which the file that is to be loaded resides.
   * @param file the string representation of the path to the file that is to be loaded, relative to
   *     the provided directory.
   * @return a new configuration loader.
   * @since 2.0.0
   */
  public static @NotNull ConfigurationLoader fromFile(
      @NotNull File directory, @NotNull String file) {
    return fromFile(new File(directory, file));
  }

  /**
   * Sets whether changes made to the {@link Configuration} should be automatically saved to file as
   * soon as they are made.
   *
   * <p>Note that enabling auto save could become very inefficient in case of frequent write
   * operations due to the blocking nature of I/O; that is why auto save is disable by default and
   * requires manual enable.
   *
   * @param autoSave {@code true} to enable auto save, {@code false} to disable it (default
   *     behaviour).
   * @return the current configuration loader itself.
   * @since 2.0.0
   */
  public @NotNull ConfigurationLoader autoSave(boolean autoSave) {
    this.autoSave = autoSave;
    return this;
  }

  /**
   * Sets the path to the resource file which contains the default configuration.
   *
   * <p>Such file must be bundled within the resulting jar to allow for automatic configuration file
   * creation when calling {@link #load()}.<br>
   * The default resource is saved if and only if the file specified with any of the {@link
   * #fromFile} method variants could not be found.<br>
   * <br>
   *
   * @param defaultResource the path to the resource file which contains the default configuration.
   * @return the current configuration loader itself.
   * @since 2.0.0
   */
  public @NotNull ConfigurationLoader setDefaultResource(@NotNull String defaultResource) {
    this.saveDefaultResource = true;
    this.defaultResource = defaultResource;
    return this;
  }

  /**
   * Loads the {@link Configuration} as specified within this configuration loader.
   *
   * @return the loaded configuration.
   * @throws IllegalArgumentException if any problem emerge when creating files and/or directories
   *     as specified.
   * @since 2.0.0
   */
  public @NotNull Configuration load() {
    if (!file.exists()) {
      // fail if file doesn't exist and save default was not requested
      if (!saveDefaultResource) {
        throw new IllegalArgumentException(
            "file should not be saved from resource but does not exist");
      }

      // create folder structure
      File parent = file.getParentFile();
      if (parent != null && !parent.exists()) {
        if (!parent.mkdirs()) {
          throw new IllegalArgumentException("could not create directory " + parent);
        }
      }

      // establish default resource
      if (defaultResource == null) {
        defaultResource = file.getName();
      }

      // copy file from resources
      if (!defaultResource.startsWith("/")) {
        defaultResource = "/".concat(defaultResource);
      }
      try (InputStream in = getClass().getResourceAsStream(defaultResource)) {
        if (in != null) {
          Files.copy(in, file.toPath());
        }
      } catch (IOException e) {
        e.printStackTrace();
      }
    }

    return new Configuration(file, autoSave);
  }
}
