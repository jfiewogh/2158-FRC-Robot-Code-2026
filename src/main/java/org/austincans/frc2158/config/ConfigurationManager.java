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
// Updated on Wednesday November 05, 2025 at 17:57:47 CST

package org.austincans.frc2158.config;

import java.io.File;
import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.jetbrains.annotations.NotNull;

/**
 * A thread-safe manager for multiple {@link Configuration} objects loaded as instructed by the
 * given {@link ConfigurationDescription} enum.
 *
 * <p>The same enum can then be used to perform various operations through this manager.
 *
 * @param <T> an enum that must implement {@link ConfigurationDescription} which is used to instruct
 *     the configuration manager on how to load configuration files.
 * @since 2.0.0
 */
// A Glowy piace :D
public final class ConfigurationManager<T extends Enum<T> & ConfigurationDescription> {

  private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();

  private final Map<@NotNull T, org.austincans.frc2158.config.Configuration> configurations;
  private final File directory;
  private final Class<T> clazz;

  /**
   * Construct a new configuration managers which loads configurations from the specified {@code
   * directory} using the given {@code description} enum.
   *
   * @param directory the directory inside which configuration files are to be loaded.
   * @param description the configuration description enum which provides instruction on how
   *     configuration files are to be loaded.
   * @since 2.0.0
   */
  public ConfigurationManager(@NotNull File directory, @NotNull Class<T> description) {
    this.configurations = new EnumMap<>(description);
    this.directory = directory;
    this.clazz = description;
  }

  /**
   * Loads the specified {@link Configuration}, if not already loaded; then returns it.
   *
   * @param configuration the configuration to be loaded.
   * @return the loaded configuration.
   * @since 2.0.0
   */
  public @NotNull Configuration load(@NotNull T configuration) {
    lock.writeLock().lock();
    try {
      Config config = makeConfig(configuration);
      ConfigurationLoader loader =
          ConfigurationLoader.fromFile(directory, config.path()).autoSave(config.autoSave());

      if (config.saveDefaultResource()) {
        loader.setDefaultResource(config.defaultResource());
      }

      Configuration loaded = loader.load();
      configurations.put(configuration, loaded);
      return loaded;
    } finally {
      lock.writeLock().unlock();
    }
  }

  /**
   * Loads all {@link Configuration}s as specified by the given {@link ConfigurationDescription}.
   *
   * @since 2.0.0
   */
  public void loadAll() {
    lock.writeLock().lock();
    try {
      for (T configuration : clazz.getEnumConstants()) {
        load(configuration);
      }
    } finally {
      lock.writeLock().unlock();
    }
  }

  /**
   * Reloads any previously loaded {@link Configuration}s.
   *
   * @since 2.2.0
   */
  public void reloadAll() {
    lock.writeLock().lock();
    try {
      for (Configuration config : configurations.values()) {
        if (config != null) {
          config.reload();
        }
      }
    } finally {
      lock.writeLock().unlock();
    }
  }

  /**
   * Unloads the specified {@link Configuration}, if already loaded.
   *
   * @param configuration the configuration to be unloaded.
   * @since 2.0.0
   */
  public void unload(@NotNull T configuration) {
    lock.writeLock().lock();
    try {
      configurations.remove(configuration);
    } finally {
      lock.writeLock().unlock();
    }
  }

  /**
   * Unloads any previously loaded {@link Configuration}.
   *
   * @since 2.0.0
   */
  public void unloadAll() {
    lock.writeLock().lock();
    try {
      configurations.clear();
    } finally {
      lock.writeLock().unlock();
    }
  }

  /**
   * Returns the specified optional {@link Configuration}, if loaded; or an empty {@link Optional}
   * otherwise.
   *
   * @param configuration the optional configuration that is to be returned if loaded.
   * @return the specified optional {@link Configuration}, if loaded.
   * @since 2.0.0
   */
  public @NotNull Optional<Configuration> getOptional(@NotNull T configuration) {
    lock.readLock().lock();
    try {
      return Optional.ofNullable(configurations.get(configuration));
    } finally {
      lock.readLock().unlock();
    }
  }

  /**
   * Returns the specified {@link Configuration} if loaded; otherwise returns the given default one.
   *
   * @param configuration the configuration that is to be returned.
   * @param def the default configuration that is to be returned if the specified one is not loaded.
   * @return the specified {@link Configuration} if loaded; or the given default one if not.
   * @since 2.0.0
   */
  public @NotNull Configuration get(@NotNull T configuration, @NotNull Configuration def) {
    // getOptional already acquires lock
    return getOptional(configuration).orElse(def);
  }

  /**
   * Returns the specified {@link Configuration} if loaded, the given default one if the first is
   * not loaded, or throws {@link NullPointerException} if none is loaded.
   *
   * @param configuration the configuration that is to be returned.
   * @param def the default configuration that is to be returned if the specified one is not loaded.
   * @return the specified {@link Configuration} if loaded; or the given default one if not.
   * @throws NullPointerException if none of the configurations are loaded when this method is
   *     invoked.
   * @since 2.0.0
   */
  public @NotNull Configuration get(@NotNull T configuration, @NotNull T def) {
    // read lock needed as two get operations could be performed and must be performed atomically
    lock.readLock().lock();
    try {
      Optional<Configuration> opt = getOptional(configuration);
      return opt.orElseGet(() -> get(def));
    } finally {
      lock.readLock().unlock();
    }
  }

  /**
   * Returns the specified {@link Configuration} if loaded; otherwise throws {@link
   * NullPointerException}.
   *
   * @param configuration the configuration that is to be returned.
   * @return the specified {@link Configuration} if loaded.
   * @throws NullPointerException if the specified configuration is not loaded when this method is
   *     invoked.
   * @since 2.0.0
   */
  public @NotNull Configuration get(@NotNull T configuration) {
    // getOptional already acquires lock
    return getOptional(configuration)
        .orElseThrow(
            () -> new NullPointerException("configuration " + configuration + " was not loaded"));
  }

  // ============================================
  //                   INTERNAL
  // ============================================

  @NotNull
  Config makeConfig(@NotNull T configuration) {
    try {
      Field field = clazz.getField(configuration.name());
      Config annotation = field.getDeclaredAnnotation(Config.class);
      return new Config() {
        @Override
        public Class<? extends Annotation> annotationType() {
          return Config.class;
        }

        @Override
        public @NotNull String path() {
          String path = annotation != null ? annotation.path() : ConfigDefaults.PATH;
          return path.isBlank() ? configuration.name().toLowerCase() + ".yml" : path;
        }

        @Override
        public @NotNull String defaultResource() {
          String defaultResource =
              annotation != null ? annotation.defaultResource() : ConfigDefaults.RESOURCE;
          return defaultResource.isBlank() ? path() : defaultResource;
        }

        @Override
        public boolean autoSave() {
          return annotation != null ? annotation.autoSave() : ConfigDefaults.AUTO_SAVE;
        }

        @Override
        public boolean saveDefaultResource() {
          return annotation != null
              ? annotation.saveDefaultResource()
              : ConfigDefaults.SAVE_DEFAULT_RESOURCE;
        }
      };
    } catch (NoSuchFieldException e) {
      throw new NullPointerException();
    }
  }
}
