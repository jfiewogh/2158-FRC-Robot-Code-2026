// Copyright (c) 2026 the ausTIN CANs
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

package org.austincans.frc2026.lidar.impl.tca9548a;

import java.util.Optional;
import org.austincans.frc2026.lidar.api.Error;
import org.austincans.frc2026.lidar.arcompat.PololuI2c;

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
