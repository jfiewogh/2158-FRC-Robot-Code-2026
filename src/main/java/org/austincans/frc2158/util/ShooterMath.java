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
// Updated on Friday January 16, 2026 at 12:36:02 CST

package org.austincans.frc2158.util;

public class ShooterMath {
  private static final double GRAVITY = 9.80665;

  private static final double DRAG_COEFF = 0.91175;
  private static final double AIR_DENSITY = 1.225; // kg/m^3
  private static final double OBJECT_MASS = 0.227; // kg

  // Calculate flywheel velocity given distance and hood angle
  public static double calculateProjectileVelocity(
      double distanceMeters, double heightDifferenceMeters, double hoodAngleRadians) {

    double cos = Math.cos(hoodAngleRadians);
    double tan = Math.tan(hoodAngleRadians);

    double numerator = GRAVITY * distanceMeters * distanceMeters;
    double denominator = 2 * cos * cos * (distanceMeters * tan - heightDifferenceMeters);

    if (denominator <= 0) return Double.NaN;

    return Math.sqrt(numerator / denominator);
  }

  // calculate projectile launch angle given (initial exitVelocity and height), (target height and distance)
  public static double calculateProjectileAngle(double initHeight, double initSpeed, double targetDistance, double targetHeight) {
    double deltaY = targetHeight - initHeight;

    double a = (GRAVITY * Math.pow(targetDistance, 2)) / (2 * Math.pow(initSpeed, 2));
    double b = targetDistance;
    double c = - (deltaY + a);
    
    double discriminant = Math.pow(b,2) - 4*a*c;
    
    if (discriminant < 0){
        return 0;
    }
    
    double tan_theta1 = (-b + Math.sqrt(discriminant)) / (2*a);
    double tan_theta2 = (-b - Math.sqrt(discriminant)) / (2*a);
    
    double angle1_rad = Math.atan(tan_theta1);
    double angle2_rad = Math.atan(tan_theta2);

    return Math.max(angle1_rad, angle2_rad);
  }

  public static double calculateDragForce(double exitVelocityMeters, double objectAreaMeters){
    double dragForceNewtons = 0;
    dragForceNewtons = (OBJECT_MASS) * (DRAG_COEFF) * (AIR_DENSITY) * Math.pow(exitVelocityMeters, 2) * (objectAreaMeters);
    return dragForceNewtons / (OBJECT_MASS);
  }

  public static double compensateForDrag(double dragForceNewtons, double time){ // TODO: Finish me
    return 0;
  }
}
