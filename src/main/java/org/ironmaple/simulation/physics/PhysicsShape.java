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
// Updated on Saturday February 21, 2026 at 23:31:39 CST

package org.ironmaple.simulation.physics;

/**
 *
 *
 * <h1>Physics Shape Abstraction</h1>
 *
 * <p>Represents a collision shape for 3D physics. This interface abstracts the underlying physics
 * engine's shape representation.
 *
 * <p>Common implementations include:
 *
 * <ul>
 *   <li>Box shapes
 *   <li>Sphere shapes
 *   <li>Cylinder shapes
 *   <li>Convex hull shapes (from mesh data)
 *   <li>Compound shapes (multiple primitives)
 * </ul>
 */
public interface PhysicsShape {

  /**
   *
   *
   * <h2>Gets the Type of This Shape.</h2>
   *
   * @return the shape type
   */
  ShapeType getType();

  /** Enumeration of supported shape types. */
  enum ShapeType {
    /** A 3D box defined by half-extents. */
    BOX,
    /** A sphere defined by radius. */
    SPHERE,
    /** A cylinder defined by radius and height. */
    CYLINDER,
    /** A capsule (cylinder with hemispherical caps). */
    CAPSULE,
    /** A convex hull defined by vertices. */
    CONVEX_HULL,
    /** A compound shape made of multiple child shapes. */
    COMPOUND,
    /** A static triangle mesh (for terrain/walls). */
    MESH
  }
}
