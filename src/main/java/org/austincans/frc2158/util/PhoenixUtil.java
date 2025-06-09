// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Signals for synchronized refresh. */
  private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];

  private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

  /** Registers a set of signals for synchronized refresh. */
  public static void registerSignals(boolean canivore, BaseStatusSignal... signals) {
    if (canivore) {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[canivoreSignals.length + signals.length];
      System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.length);
      System.arraycopy(signals, 0, newSignals, canivoreSignals.length, signals.length);
      canivoreSignals = newSignals;
    } else {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
      System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
      System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
      rioSignals = newSignals;
    }
  }

  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }

  /** Refresh all registered signals. */
  public static void refreshAll() {
    if (canivoreSignals.length > 0) {
      BaseStatusSignal.refreshAll(canivoreSignals);
    }
    if (rioSignals.length > 0) {
      BaseStatusSignal.refreshAll(rioSignals);
    }
  }
}
