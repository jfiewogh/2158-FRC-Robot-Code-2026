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

package org.austincans.frc2026.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Tracker that activates only when a button is pressed twice quickly. */
public class DoublePressTracker {
  public static final double maxLengthSecs =
      0.4; // How long after the first press does the second need to occur?

  private final Trigger trigger;
  private final Timer resetTimer = new Timer();
  private DoublePressState state = DoublePressState.IDLE;

  public static Trigger doublePress(Trigger baseTrigger) {
    var tracker = new DoublePressTracker(baseTrigger);
    return new Trigger(tracker::get);
  }

  private DoublePressTracker(Trigger baseTrigger) {
    trigger = baseTrigger;
  }

  private boolean get() {
    boolean pressed = trigger.getAsBoolean();
    switch (state) {
      case IDLE:
        if (pressed) {
          state = DoublePressState.FIRST_PRESS;
          resetTimer.reset();
          resetTimer.start();
        }
        break;
      case FIRST_PRESS:
        if (!pressed) {
          if (resetTimer.hasElapsed(maxLengthSecs)) {
            reset();
          } else {
            state = DoublePressState.FIRST_RELEASE;
          }
        }
        break;
      case FIRST_RELEASE:
        if (pressed) {
          state = DoublePressState.SECOND_PRESS;
        } else if (resetTimer.hasElapsed(maxLengthSecs)) {
          reset();
        }
        break;
      case SECOND_PRESS:
        if (!pressed) {
          reset();
        }
    }
    return state == DoublePressState.SECOND_PRESS;
  }

  private void reset() {
    state = DoublePressState.IDLE;
    resetTimer.stop();
  }

  private enum DoublePressState {
    IDLE,
    FIRST_PRESS,
    FIRST_RELEASE,
    SECOND_PRESS
  }
}
