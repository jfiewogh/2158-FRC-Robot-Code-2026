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
// Updated on Saturday January 31, 2026 at 16:59:54 CST

package org.ironmaple.simulation.physics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;

/**
 *
 *
 * <h1>Physics Backend Abstraction</h1>
 *
 * <p>This interface provides a unified API for physics simulation backends. It supports both 2D
 * (dyn4j) and 3D (Bullet) physics engines through a common interface.
 *
 * <p>The backend handles:
 *
 * <ul>
 *   <li>World management (initialization, stepping, shutdown)
 *   <li>Body creation and removal
 *   <li>Collision detection
 *   <li>Raycasting (3D only)
 * </ul>
 */
public interface PhysicsBackend {

  /**
   *
   *
   * <h2>Returns Whether This Backend is 3D.</h2>
   *
   * @return true if this is a 3D physics backend, false for 2D
   */
  boolean is3D();

  /**
   *
   *
   * <h2>Initializes the Physics Backend.</h2>
   */
  void initialize();

  /**
   *
   *
   * <h2>Shuts Down the Physics Backend.</h2>
   */
  void shutdown();

  /**
   *
   *
   * <h2>Steps the Physics Simulation.</h2>
   *
   * @param deltaTime the time step
   */
  void step(Time deltaTime);

  /**
   *
   *
   * <h2>Adds a Static Obstacle to the World.</h2>
   *
   * <p>For 2D backends, the z-coordinate is ignored.
   *
   * @param halfExtents the half-extents of the box obstacle
   * @param pose the pose of the obstacle
   * @return an opaque handle to the created body
   */
  Object addStaticBox(Translation3d halfExtents, Pose3d pose);

  /**
   *
   *
   * <h2>Adds a Static Line Segment Obstacle (2D Only).</h2>
   *
   * <p>For 3D backends, this creates a thin box approximation.
   *
   * @param start the start point
   * @param end the end point
   * @return an opaque handle to the created body
   */
  Object addStaticLine(Translation2d start, Translation2d end);

  /**
   *
   *
   * <h2>Removes a Body from the World.</h2>
   *
   * @param bodyHandle the handle returned from an add method
   */
  void removeBody(Object bodyHandle);

  /**
   *
   *
   * <h2>Removes All Bodies from the World.</h2>
   */
  void removeAllBodies();

  /**
   *
   *
   * <h2>Performs a Raycast (3D Only).</h2>
   *
   * <p>For 2D backends, this always returns empty.
   *
   * @param origin the ray origin
   * @param direction the ray direction
   * @param maxDistance the maximum distance
   * @return the raycast result, or empty if no hit
   */
  default Optional<PhysicsEngine.RaycastResult> raycast(
      Translation3d origin, Translation3d direction, double maxDistance) {
    return Optional.empty();
  }

  /**
   *
   *
   * <h2>Sets the Gravity.</h2>
   *
   * <p>For 2D backends, only supports zero gravity (horizontal plane).
   *
   * @param gravity the gravity vector (m/s²)
   */
  void setGravity(Translation3d gravity);

  /**
   * Checks if this backend is running in threaded mode.
   *
   * @return true if physics runs on a dedicated thread
   */
  default boolean isThreaded() {
    return false;
  }

  /**
   * Gets the threaded physics proxy for async operations.
   *
   * @return the proxy, or null if not in threaded mode
   */
  default org.ironmaple.simulation.physics.threading.ThreadedPhysicsProxy getThreadedProxy() {
    return null;
  }

  /**
   * Flushes queued inputs to the physics thread.
   *
   * <p>Only has effect in threaded mode. Call at end of simulation period.
   */
  default void flushInputs() {}

  /**
   * Pulls the latest state from the physics thread.
   *
   * <p>Only has effect in threaded mode. Call at start of simulation period.
   */
  default void pullLatestState() {}
}
