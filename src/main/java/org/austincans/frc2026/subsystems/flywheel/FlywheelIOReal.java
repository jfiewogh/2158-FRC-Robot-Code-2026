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

package org.austincans.frc2026.subsystems.flywheel;

import org.austincans.lib.sensors.SensorIO;

import edu.wpi.first.units.Units;

// Includes the sensors used in a real flywheel
public abstract class FlywheelIOReal implements FlywheelIO {

  private SensorIO io1;
  private SensorIO io2;

  public FlywheelIOReal(SensorIO io){
    io1 = io;
    io2 = io;
  }

  public void updateInputs(FlywheelIOInputs inputs){
    io1.updateInputs(inputs.inputs1);
    io2.updateInputs(inputs.inputs2);
  }

  @Override
  public boolean readyToShoot() {
    return io1.getMeasurement() < FlywheelConstants.maxDistFromLasarCan1;
  }

  @Override
  public boolean isShooting() {
    return io2.getMeasurement() < FlywheelConstants.maxDistFromLasarCan2;
  }

  // TODO: complete
  @Override
  public boolean noGamePiecesToShoot() {
    return false;
  }
}
