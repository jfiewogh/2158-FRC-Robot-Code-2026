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

package org.austincans.frc2158.subsystems.selfcheck;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPigeon2 implements SelfChecking {
  private final String label;
  private final Pigeon2 pigeon;

  public SelfCheckingPigeon2(String label, Pigeon2 pigeon) {
    this.label = label;
    this.pigeon = pigeon;
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();

    if (this.pigeon.getFault_Hardware(true).getValue()) {
      faults.add(new HardwareFault("PIGE", String.format("[%s]: Hardware fault detected", label)));
    }
    if (this.pigeon.getFault_BootDuringEnable(true).getValue()) {
      faults.add(
          new HardwareFault("PIGE", String.format("[%s]: Device booted while enabled", label)));
    }
    if (this.pigeon.getFault_BootIntoMotion(true).getValue()) {
      faults.add(
          new HardwareFault("PIGE", String.format("[%s]: Device booted while in motion", label)));
    }
    if (this.pigeon.getFault_BootupGyroscope(true).getValue()) {
      faults.add(new HardwareFault("PIGE", String.format("[%s]: Gyro fault detected", label)));
    }
    if (this.pigeon.getFault_BootupAccelerometer().getValue()) {
      faults.add(
          new HardwareFault("PIGE", String.format("[%s]: Accelerometer fault detected", label)));
    }

    StatusSignal<Integer> faultsError =
        pigeon.getFaultField(
            true); // fills faults with the current Pigeon 2 faults; returns the last error
    // generated
    if (faultsError.getValue() != 0) {
      faults.add(
          new HardwareFault(
              "PIGE", String.format("[%s]: Error Code (%s)", label, faultsError.getValue())));
    }

    return faults;
  }
}
