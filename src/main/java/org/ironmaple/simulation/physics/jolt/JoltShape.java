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

package org.ironmaple.simulation.physics.jolt;

import com.github.stephengold.joltjni.readonly.ConstShape;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Jolt Physics Shape Implementation</h1>
 *
 * <p>Implements the {@link PhysicsShape} interface by wrapping a Jolt {@link ConstShape}.
 */
public class JoltShape implements PhysicsShape {
  private final ConstShape shape;
  private final ShapeType type;

  /**
   *
   *
   * <h2>Creates a JoltShape Wrapper.</h2>
   *
   * @param shape the underlying Jolt shape
   * @param type the shape type
   */
  public JoltShape(ConstShape shape, ShapeType type) {
    this.shape = shape;
    this.type = type;
  }

  /**
   *
   *
   * <h2>Gets the Underlying Jolt Shape.</h2>
   *
   * @return the Jolt shape
   */
  public ConstShape getShape() {
    return shape;
  }

  @Override
  public ShapeType getType() {
    return type;
  }
}
