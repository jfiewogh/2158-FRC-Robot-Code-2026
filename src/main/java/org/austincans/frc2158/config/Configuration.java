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
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.representer.Representer;

/**
 * A thread-safe configuration data store which supports many read/write operations.
 *
 * <p>Configuration is loaded from file as specified through a {@link ConfigurationLoader}.<br>
 * Any read-write operation is completely thread-safe, including {@link #save()} and {@link
 * #reload()}.
 *
 * @since 2.0.0
 */
public final class Configuration implements ConfigurationSection {

  private final ReentrantReadWriteLock lock;
  private final boolean autoSave;
  private final File file;
  private final Yaml yaml;

  private ConfigurationSectionImpl section;

  Configuration(@NotNull File file, boolean autoSave) {
    this.lock = new ReentrantReadWriteLock(true);
    this.autoSave = autoSave;
    this.file = file;

    DumperOptions options = new DumperOptions();
    options.setDefaultFlowStyle(DumperOptions.FlowStyle.BLOCK);
    Representer representer = new ConfigurationRepresenter(options);
    this.yaml = new Yaml(representer, options);

    load();
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public boolean contains(@NotNull String path) {
    return section.contains(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull Object get(@NotNull String path) {
    return section.get(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public <T> @Nullable T get(@NotNull String path, @Nullable T def) {
    return section.get(path, def);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull List<@Nullable Object> getList(@NotNull String path) {
    return section.getList(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public void set(@NotNull String path, @Nullable Object value) {
    section.set(path, value);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public void unset(@NotNull String path) {
    section.unset(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull ConfigurationSection createSection(@NotNull String path) {
    return section.createSection(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull ConfigurationSection getOrCreateSection(@NotNull String path) {
    return section.getOrCreateSection(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull ConfigurationSection getSection(@NotNull String path) {
    return section.getSection(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @Nullable ConfigurationSection getSection(
      @NotNull String path, @Nullable ConfigurationSection def) {
    return section.getSection(path, def);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull Set<@NotNull String> getKeys(@NotNull TraversalMode traversalMode) {
    return section.getKeys(traversalMode);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public byte getByte(@NotNull String path) {
    return section.getByte(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public byte getByte(@NotNull String path, byte def) {
    return section.getByte(path, def);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull List<@NotNull Byte> getByteList(@NotNull String path) {
    return section.getByteList(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public short getShort(@NotNull String path) {
    return section.getShort(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public short getShort(@NotNull String path, short def) {
    return section.getShort(path, def);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull List<@NotNull Short> getShortList(@NotNull String path) {
    return section.getShortList(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public int getInt(@NotNull String path) {
    return section.getInt(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public int getInt(@NotNull String path, int def) {
    return section.getInt(path, def);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull List<@NotNull Integer> getIntList(@NotNull String path) {
    return section.getIntList(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public long getLong(@NotNull String path) {
    return section.getLong(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public long getLong(@NotNull String path, long def) {
    return section.getLong(path, def);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull List<@NotNull Long> getLongList(@NotNull String path) {
    return section.getLongList(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public float getFloat(@NotNull String path) {
    return section.getFloat(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public float getFloat(@NotNull String path, float def) {
    return section.getFloat(path, def);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull List<@NotNull Float> getFloatList(@NotNull String path) {
    return section.getFloatList(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public double getDouble(@NotNull String path) {
    return section.getDouble(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public double getDouble(@NotNull String path, double def) {
    return section.getDouble(path, def);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull List<@NotNull Double> getDoubleList(@NotNull String path) {
    return section.getDoubleList(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public boolean getBoolean(@NotNull String path) {
    return section.getBoolean(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public boolean getBoolean(@NotNull String path, boolean def) {
    return section.getBoolean(path, def);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull List<@NotNull Boolean> getBooleanList(@NotNull String path) {
    return section.getBooleanList(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull String getString(@NotNull String path) {
    return section.getString(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @Nullable String getString(@NotNull String path, @Nullable String def) {
    return section.getString(path, def);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull List<@NotNull String> getStringList(@NotNull String path) {
    return section.getStringList(path);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public <T extends Enum<T>> @NotNull T getEnum(@NotNull String path, @NotNull Class<T> enumClass) {
    return section.getEnum(path, enumClass);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public <T extends Enum<T>> @Nullable T getEnum(
      @NotNull String path, @NotNull Class<T> enumClass, @Nullable T def) {
    return section.getEnum(path, enumClass, def);
  }

  /**
   * {@inheritDoc}
   *
   * @since 2.0.0
   */
  @Override
  public @NotNull <T extends Enum<T>> List<@NotNull T> getEnumList(
      @NotNull String path, @NotNull Class<T> enumClass) {
    return section.getEnumList(path, enumClass);
  }

  /**
   * Saves the configuration to the source file.
   *
   * @since 2.0.0
   */
  public void save() {
    writeLock().lock();
    try (Writer writer =
        new OutputStreamWriter(new FileOutputStream(file), StandardCharsets.UTF_8)) {
      yaml.dump(section.getData(), writer);
    } catch (IOException e) {
      e.printStackTrace();
    } finally {
      writeLock().unlock();
    }
  }

  /**
   * Reloads the configuration from file, discarding any unsaved changes and reflecting any changes
   * that was made to the file.
   *
   * @since 2.0.0
   */
  public void reload() {
    // load already acquires lock
    load();
  }

  /**
   * Returns the read lock associated to this configuration.
   *
   * <p>Locks should be used with extreme care.<br>
   * Please, refer to the official javadocs for further information:
   *
   * <ul>
   *   <li>{@link ReentrantReadWriteLock}
   *   <li>{@link ReentrantReadWriteLock.ReadLock}
   * </ul>
   *
   * @return the read lock associated to the whole {@link Configuration}.
   * @since 2.0.0
   */
  public @NotNull ReentrantReadWriteLock.ReadLock readLock() {
    return lock.readLock();
  }

  /**
   * Returns the write lock associated to this configuration.
   *
   * <p>Locks should be used with extreme care.<br>
   * Please, refer to the official javadocs for further information:
   *
   * <ul>
   *   <li>{@link ReentrantReadWriteLock}
   *   <li>{@link ReentrantReadWriteLock.WriteLock}
   * </ul>
   *
   * @return the read lock associated to the whole {@link Configuration}.
   * @since 2.0.0
   */
  public @NotNull ReentrantReadWriteLock.WriteLock writeLock() {
    return lock.writeLock();
  }

  // ============================================
  //                   INTERNAL
  // ============================================

  void autoSave() {
    if (autoSave) {
      save();
    }
  }

  private void load() {
    writeLock().lock();
    try {
      try (InputStream in = new FileInputStream(file)) {
        Map<String, Object> data = yaml.load(in);
        section = new ConfigurationSectionImpl(this, data);
      } catch (IOException e) {
        e.printStackTrace();
      }
    } finally {
      writeLock().unlock();
    }
  }
}
