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

public interface GyroIO extends DeviceIO<GyroIO.GyroIOInputs> {
  public class GyroIOInputs {
    private Angle yawPosition;
    private Angle pitchPosition;
    private Angle rollPosition;
    private AngularVelocity yawVelocity;
    private AngularVelocity pitchVelocity;
    private AngularVelocity rollVelocity;

    public void setYawPosition(Angle yawPosition) {
      this.yawPosition = yawPosition;
    }
    public void setPitchPosition(Angle pitchPosition) {
      this.pitchPosition = pitchPosition;
    }
    public void setRollPosition(Angle rollPosition) {
      this.rollPosition = rollPosition;
    }
    public void setYawVelocity(AngularVelocity yawVelocity) {
      this.yawVelocity = yawVelocity;
    }
    public void setPitchVelocity(AngularVelocity pitchVelocity) {
      this.pitchVelocity = pitchVelocity;
    }
    public void setRollVelocity(AngularVelocity rollVelocity) {
      this.rollVelocity = rollVelocity;
    }

    public Angle getYawPosition() {
      return yawPosition;
    }
    public Angle getPitchPosition() {
      return pitchPosition;
    }
    public Angle getRollPosition() {
      return rollPosition;
    }
    public AngularVelocity getYawVelocity() {
      return yawVelocity;
    }
    public AngularVelocity getPitchVelocity() {
      return pitchVelocity;
    }
    public AngularVelocity getRollVelocity() {
      return rollVelocity;
    }
  }
}
