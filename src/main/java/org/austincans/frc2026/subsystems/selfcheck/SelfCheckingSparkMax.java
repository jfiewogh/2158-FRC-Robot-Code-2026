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

package org.austincans.frc2026.subsystems.selfcheck;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingSparkMax implements SelfChecking {
  private final String label;
  private SparkMax spark;

  public SelfCheckingSparkMax(String label, SparkMax spark) {
    this.label = label;
    this.spark = spark;
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    ArrayList<HardwareFault> faults = new ArrayList<>();

    REVLibError err = spark.getLastError();
    if (err != REVLibError.kOk) {
      faults.add(
          new HardwareFault(
              "SPRK" + String.valueOf(spark.getDeviceId()),
              String.format("[%s]: Error: %s", label, err.name())));
    }

    return faults;
  }
}
