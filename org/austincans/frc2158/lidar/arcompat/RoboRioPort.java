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

package org.austincans.frc2158.lidar.arcompat;

/** Ports available on the RoboRIO board. */
public enum RoboRioPort implements Port {
    /** The I2C port on the RoboRIO's board. */
    ONBOARD(0),
    /** The I2C port on the MXP output. */
    MXP(1);

    private final int port;

    RoboRioPort(int port) {
        this.port = port;
    }

    @Override
    public int value() {
        return port;
    }
}
