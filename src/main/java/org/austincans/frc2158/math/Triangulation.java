// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.math;

import org.austincans.frc2158.util.ConfigUtil;

/*
 * @author Austin Cans Robotics Team 2158
 */
public class Triangulation {

  /*
   * Alignment Front
   * P0 the target point
   * P1 the left point
   * P2 the right point
   * p0p1Distance: Distance from P1 to P0
   * p1p2Distance: Distance from P1 to P2 (Baseline length)
   * p0p2Distance: Distance from P2 to P0
   * angleAlpha: the angle between P1P0 and P1P2
   * angleBeta: the angle between P2P0 and P2P1
   * angleGamma: the angle between P0P1 and P0P2
   * p0X: x-coordinates of the point P0
   * p0Y: y-coordinates of the point P0
   *
   * Calculations:
   * p0X = (Math.pow(p0p1Distance,2) + Math.pow(p1p2Distance,2) - Math.pow(p0p2Distance,2)) / (2 * p1p2Distance)
   * p0Y = Math.sqrt(Math.pow(p0p1Distance,2) - Math.pow(p0X,2))
   */
  private Coordinate p0 = new Coordinate();
  private Coordinate p1 = new Coordinate();
  private Coordinate p2 = new Coordinate();
  private Double p0p1Distance = null;
  private Double p0p2Distance = null;
  private Double p1p2Distance = null;
  private Double angleAlpha = null;
  private Double angleBeta = null;
  private Double angleGamma = null;

  public Triangulation(Double p1Distance, Double p2Distance) {
    this.p0p1Distance = p1Distance;
    this.p0p2Distance = p2Distance;
    setP1P2Distance(null);
    setPoints();
    setAngles();
  }

  // Set P0 X and Y coordinates. P0 is the target
  public void setP0XY() {
    this.p0.setCoordinateX(getP0X());
    this.p0.setCoordinateY(getP0Y());
  }

  // Get P0 coordinates
  public Coordinate getP0XY() {
    return this.p0;
  }

  // Set P0 X and Y coordinates. P0 is the origin VLF at Left
  public void setP1XY() {
    this.p1.setCoordinateX(0.0);
    this.p1.setCoordinateY(0.0);
  }

  // Get P1 coordinates
  public Coordinate getP1XY() {
    return this.p1;
  }

  // Set P2 X and Y coordinates. P2 is the second VLF at Right
  public void setP2XY(Double p2X) {
    // If coordinate is null, take value from config file or set default to DEV board dimension
    if (p2X == null) {
      // P2 X Coordinate define in config file
      p2X = ConfigUtil.getConfigDouble("VLF_P2_X", 50.0);
    }
    this.p2.setCoordinateX(p2X);
    this.p2.setCoordinateY(0.0);
  }

  // Get P2 coordinates
  public Coordinate getP2XY() {
    return this.p2;
  }

  public void setPoints() {
    setP1XY();
    setP2XY(null);
    setP0XY();
  }

  /*
   * P1 VLF Left set Distance to target P0
   */
  public boolean setP1P0Distance(Double p0p1Distance) {
    if (p0p1Distance == null) {
      return false;
    }
    this.p0p1Distance = p0p1Distance;
    return true;
  }

  // Get Distance between target P0 and origin VLF at Left
  public Double getP0P1Distance() {
    if (this.p0p1Distance == null) {
      this.p0p1Distance = 0.0; // initialize to default 0.0
    }
    return this.p0p1Distance;
  }

  /*
   * P2 VLF Right set Distance to Target P0
   */
  public boolean setP2P0Distance(Double p0p2Distance) {
    if (p0p2Distance == null) {
      return false;
    }
    this.p0p2Distance = p0p2Distance;
    return true;
  }

  // Get Distance between target P0 and VLF at Right
  public Double getP2P0Distance() {
    if (this.p0p2Distance == null) {
      this.p0p2Distance = 0.0; // initialize to default 0.0
    }
    return this.p0p2Distance;
  }

  /*
   * Set Distance between the 2 VLFs
   */
  public void setP1P2Distance(Double p1p2Distance) {
    if (p1p2Distance == null) {
      this.p1p2Distance = ConfigUtil.getConfigDouble("VLF_P2_X", 50.0);
    } else {
      this.p1p2Distance = p1p2Distance;
    }
  }

  // Get Distance between the 2 VLFs
  public Double getP1P2Distance() {
    if (this.p1p2Distance == null) {
      this.p1p2Distance = this.p2.getCoordinateX(); // initialize to default config entry
    }
    return this.p1p2Distance;
  }

  // Angle Alpha is the angle between P1P0 and P1P2
  public void setAngleAlpha() {
    // A2 = B2 + C2 - 2BC COS ALPHA
    // A is p1p2Distance
    // B is p0p2Distance
    // C is p0p1Distance
    // COS ALPHA = -(A2 - B2 - C2) / 2BC
    // ALPHA = ACOS (COS ALPHA)
    double alpha =
        Math.acos(
            (-(Math.pow(getP1P2Distance(), 2)
                    - Math.pow(getP2P0Distance(), 2)
                    - Math.pow(getP0P1Distance(), 2))
                / (2 * getP2P0Distance() * getP0P1Distance())));
    this.angleAlpha = alpha;
  }

  // Get Angle Alpha value in Degrees
  public double getAngleAlphaDegrees() {
    double alpha = Math.toDegrees(this.angleAlpha);
    return CANMath.round(alpha, 2);
  }

  // Get Angle Alpha value in Radians
  public double getAngleAlphaRadians() {
    double alpha = Math.toRadians(this.angleAlpha);
    return CANMath.round(alpha, 3);
  }

  public void setAngleBeta() {
    // B2 = A2 + C2 - 2AC COS BETA
    // A is p1p2Distance
    // B is p0p2Distance
    // C is p0p1Distance
    // COS BETA = -(B2 - A2 - C2) / 2AC
    // BETA = ACOS (COS BETA)
    double beta =
        Math.acos(
            (-(Math.pow(getP2P0Distance(), 2)
                    - Math.pow(getP1P2Distance(), 2)
                    - Math.pow(getP0P1Distance(), 2))
                / (2 * getP1P2Distance() * getP0P1Distance())));
    this.angleBeta = beta;
  }

  // Get Angle Alpha value in Degrees
  public double getAngleBetaDegrees() {
    double beta = Math.toDegrees(this.angleBeta);
    return CANMath.round(beta, 2);
  }

  // Get Angle Alpha value in Radians
  public double getAngleBetaRadians() {
    double beta = Math.toRadians(this.angleBeta);
    return CANMath.round(beta, 3);
  }

  public void setAngleGamma() {
    // C2 = A2 + B2 - 2AB COS GAMMA
    // A is p1p2Distance
    // B is p0p2Distance
    // C is p0p1Distance
    // COS GAMMA = -(C2 - A2 - B2) / 2AB
    // GAMMA = ACOS (COS GAMMA)
    double gamma =
        Math.acos(
            (-(Math.pow(getP0P1Distance(), 2)
                    - Math.pow(getP1P2Distance(), 2)
                    - Math.pow(getP2P0Distance(), 2))
                / (2 * getP1P2Distance() * getP2P0Distance())));
    this.angleGamma = gamma;
  }

  // Get Angle Alpha value in Degrees
  public double getAngleGammaDegrees() {
    double gamma = Math.toDegrees(this.angleGamma);
    return CANMath.round(gamma, 2);
  }

  // Get Angle Alpha value in Radians
  public double getAngleGammaRadians() {
    double gamma = Math.toRadians(this.angleGamma);
    return CANMath.round(gamma, 3);
  }

  // Set the 3 angles
  public void setAngles() {
    setAngleAlpha();
    setAngleBeta();
    setAngleGamma();
  }

  /*
   * get Target X coordinate
   */
  public double getP0X() {
    var p0X =
        (Math.pow(getP0P1Distance(), 2)
                + Math.pow(getP1P2Distance(), 2)
                - Math.pow(getP2P0Distance(), 2))
            / (2 * getP1P2Distance());
    return CANMath.round(p0X, 2);
  }

  /*
   * get Target Y coordinate
   */
  public double getP0Y() {
    var p0Y = Math.sqrt(Math.pow(getP0P1Distance(), 2) - Math.pow(getP0X(), 2));
    return CANMath.round(p0Y, 2);
  }
}

class Coordinate {
  private double X;
  private double Y;

  public void setCoordinateX(double x) {
    X = x;
  }

  public double getCoordinateX() {
    return X;
  }

  public void setCoordinateY(double y) {
    Y = y;
  }

  public double getCoordinateY() {
    return Y;
  }
}
