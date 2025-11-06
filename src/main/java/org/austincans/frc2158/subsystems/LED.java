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
// Updated on Wednesday November 05, 2025 at 18:44:58 CST

package org.austincans.frc2158.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import static java.util.Map.entry;    

public class LED extends SubsystemBase {
  private final Spark color = new Spark(0);
  Map<String, Double> ledColorList = Map.ofEntries(
    entry("Blue", 0.87),
    entry("Red", 0.61),
    entry("Gold", 0.67),
    entry("Yellow", 0.69),
    entry("Green", 0.77),
    entry("Black", 0.99),
    entry("White", 0.93)
  );
  
  /** Creates a new LED. */
  public LED() {
   
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setColor(String color) {
    Double colorFromList = ledColorList.get(color);
    color.set(colorFromList);
  }
}
