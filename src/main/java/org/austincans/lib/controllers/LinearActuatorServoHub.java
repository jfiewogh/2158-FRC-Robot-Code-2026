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

package org.austincans.frc2026.controllers;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LinearActuatorServoHub {
  private InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
  private ServoHub hub = ServoHubController.getInstance().hub;
  private ServoChannel linearActuator;

  // L16-100-35-6-R
  public LinearActuatorServoHub(int linearActuatorDeviceID) {
    linearActuator = hub.getServoChannel(ServoChannel.ChannelId.fromInt(linearActuatorDeviceID));
    linearActuator.setEnabled(true);

    // Position Percentage
    map.put(0.0, 1000.0);
    map.put(100.0, 2000.0);
  }

  public void setPulseWidth(int width) {
    linearActuator.setEnabled(true);
    linearActuator.setPowered(true);
    linearActuator.setPulseWidth(width);
  }

  public void disable() {
    linearActuator.setEnabled(false);
    linearActuator.setPowered(false);
  }

  public void setDistance(double percent) {
    linearActuator.setPulseWidth(Integer.parseInt(map.get(percent).toString()));
  }
}
