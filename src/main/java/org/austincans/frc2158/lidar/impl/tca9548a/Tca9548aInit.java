// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl.tca9548a;

import java.util.Optional;
import org.austincans.frc2158.lidar.api.Error;
import org.austincans.frc2158.lidar.arcompat.PololuI2c;

@SuppressWarnings("unused")
class Tca9548aInit {

  private final Tca9548aI2c target;
  private final PololuI2c i2c;

  Tca9548aInit(Tca9548aI2c target) {
    this.target = target;
    this.i2c = target.getI2c();
  }

  public Optional<Error<?>> initialize() {
    throw new UnsupportedOperationException("Unimplemented method 'initialize'");
  }
}
