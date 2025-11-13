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
// Updated on Thursday November 13, 2025 at 17:43:29 CST

package org.austincans.frc2158.util;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.nio.ByteBuffer;
import java.util.HashSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** Utility class to log the list of NetworkTables clients. */
public class NTClientLogger {
  private static final String tableName = "NTClients/";
  private static Set<String> lastRemoteIds = new HashSet<>();
  private static ByteBuffer intBuffer = ByteBuffer.allocate(4);

  private NTClientLogger() {}

  public static void log() {
    ConnectionInfo[] connections = NetworkTableInstance.getDefault().getConnections();
    Set<String> remoteIds = new HashSet<>();

    // Log data for connected clients
    for (int i = 0; i < connections.length; i++) {
      lastRemoteIds.remove(connections[i].remote_id);
      remoteIds.add(connections[i].remote_id);
      Logger.recordOutput(tableName + connections[i].remote_id + "/Connected", true);
      Logger.recordOutput(
          tableName + connections[i].remote_id + "/IPAddress", connections[i].remote_ip);
      Logger.recordOutput(
          tableName + connections[i].remote_id + "/RemotePort", connections[i].remote_port);
      Logger.recordOutput(
          tableName + connections[i].remote_id + "/LastUpdate", connections[i].last_update);
      intBuffer.rewind();
      Logger.recordOutput(
          tableName + connections[i].remote_id + "/ProtocolVersion",
          intBuffer.putInt(connections[i].protocol_version).array());
    }

    // Mark disconnected clients
    for (var remoteId : lastRemoteIds) {
      Logger.recordOutput(tableName + remoteId + "/Connected", false);
    }
    lastRemoteIds = remoteIds;
  }
}
