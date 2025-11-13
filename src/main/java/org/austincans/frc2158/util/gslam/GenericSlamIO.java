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

package org.austincans.frc2158.util.gslam;

import org.littletonrobotics.junction.AutoLog;

public interface GenericSlamIO {
  @AutoLog
  class GenericSlamIOInputs {
    public GenericSlamIOData data = new GenericSlamIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record GenericSlamIOData(
      boolean motorConnected,
      double positionRads,
      double velocityRadsPerSec,
      double appliedVoltage,
      double supplyCurrentAmps,
      double torqueCurrentAmps,
      double tempCelsius) {}

  /** Update the inputs. */
  default void updateInputs(GenericSlamIOInputs inputs) {}

  /** Run slam elevator at amps */
  default void runTorqueCurrent(double amps) {}

  /** Stop slam elevator */
  default void stop() {}

  /** Enable or disable brake mode on the elevator motor. */
  default void setBrakeMode(boolean enable) {}
}
