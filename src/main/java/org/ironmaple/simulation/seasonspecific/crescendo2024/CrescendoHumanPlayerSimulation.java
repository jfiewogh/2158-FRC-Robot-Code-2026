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
//
// Updated on Sunday February 22, 2026 at 14:08:53 CST

package org.ironmaple.simulation.seasonspecific.crescendo2024;

import static org.ironmaple.utils.LegacyFieldMirroringUtils2024.toCurrentAllianceTranslation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class CrescendoHumanPlayerSimulation implements SimulatedArena.Simulatable {
  private static final Translation2d BLUE_SOURCE_POSITION = new Translation2d(15.6, 0.8);
  private double previousThrowTimeSeconds = 0;
  private final Arena2024Crescendo arena;

  public CrescendoHumanPlayerSimulation(Arena2024Crescendo arena) {
    this.arena = arena;
  }

  @Override
  public void simulationSubTick(int subTickNum) {
    if (!DriverStation.isTeleopEnabled()) return;

    if (Timer.getFPGATimestamp() - previousThrowTimeSeconds < 1) return;

    final Translation2d sourcePosition = toCurrentAllianceTranslation(BLUE_SOURCE_POSITION);
    /*
     * if there is any game-piece 0.5 meters within the human player station, we
     * don't throw a new note
     */
    for (GamePieceOnFieldSimulation gamePiece : arena.gamePiecesOnField())
      if (gamePiece instanceof CrescendoNoteOnField
          && gamePiece.getPoseOnField().getTranslation().getDistance(sourcePosition) < 1) return;

    /* otherwise, place a note */
    arena.getGamePieceManager().spawnOnField(new CrescendoNoteOnField(sourcePosition));
    previousThrowTimeSeconds = Timer.getFPGATimestamp();
  }
}
