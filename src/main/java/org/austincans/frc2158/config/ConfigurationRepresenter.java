// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.config;

import org.jetbrains.annotations.NotNull;
import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.representer.Representer;

final class ConfigurationRepresenter extends Representer {

  ConfigurationRepresenter(@NotNull DumperOptions options) {
    super(options);
    this.representers.put(
        ConfigurationSectionImpl.class,
        data -> {
          ConfigurationSectionImpl section = (ConfigurationSectionImpl) data;
          section.getRoot().writeLock().lock();
          try {
            return represent(section.getData());
          } finally {
            section.getRoot().writeLock().unlock();
          }
        });
  }
}
