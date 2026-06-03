package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.SparkIO.SparkControlSlot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Current;

public class SparkConfig extends MotorConfig<SparkControlSlot> {
   public ControlConfig slot3Config = new ControlConfig();
   public MotorType type;
   public Current smartCurrentStallLimit;

   /** If true, follower should move in same direction as leader. */
   public boolean isFollower;
   public int leaderCanId; 
   /** If true, follower should move in opposite direction of leader. */
   public boolean invertFollowerOutput;
}
