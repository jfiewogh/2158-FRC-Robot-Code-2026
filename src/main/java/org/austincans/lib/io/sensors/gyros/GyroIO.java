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

package org.austincans.lib.io.sensors.gyros;

import org.austincans.lib.io.DeviceIO;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO<InputsType extends GyroIO.GyroIOInputs> extends DeviceIO<InputsType> {
  public class GyroIOInputs {
    protected boolean connected;
    protected Angle yawPosition;
    protected Angle pitchPosition;
    protected Angle rollPosition;
    protected AngularVelocity yawVelocity;
    protected AngularVelocity pitchVelocity;
    protected AngularVelocity rollVelocity;

    public boolean connected() {
      return connected;
    }
    public Angle yawPosition() {
      return yawPosition;
    }
    public Angle pitchPosition() {
      return pitchPosition;
    }
    public Angle rollPosition() {
      return rollPosition;
    }
    public AngularVelocity yawVelocity() {
      return yawVelocity;
    }
    public AngularVelocity pitchVelocity() {
      return pitchVelocity;
    }
    public AngularVelocity rollVelocity() {
      return rollVelocity;
    }
  }
}
