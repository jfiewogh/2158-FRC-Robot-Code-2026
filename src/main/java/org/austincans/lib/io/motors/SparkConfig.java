package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.SparkIO.SparkControlSlot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class SparkConfig extends MotorConfig {
   public SlotConfig slot0Config = new SlotConfig(SparkControlSlot.SLOT_0);
   public SlotConfig slot1Config = new SlotConfig(SparkControlSlot.SLOT_1);
   public SlotConfig slot2Config = new SlotConfig(SparkControlSlot.SLOT_2);
   public SlotConfig slot3Config = new SlotConfig(SparkControlSlot.SLOT_3);

   public Current smartCurrentStallLimit;

   /** If true, follower should move in same direction as leader. */
   public boolean isFollower;
   public int leaderCanId; 
   /** If true, follower should move in opposite direction of leader. */
   public boolean invertFollowerOutput;

   public static class SlotConfig {
      public SparkControlSlot controlSlot;
      public ControlConfig controlConfig = new ControlConfig();
      public Angle allowedMechanismClosedLoopError;
      
      public AngularVelocity maxMotionMechanismCruiseVelocity;
      public AngularAcceleration maxMotionMechanismMaxAcceleration;

      public SlotConfig(SparkControlSlot controlSlot) {
         this.controlSlot = controlSlot;
      }
   }
}
