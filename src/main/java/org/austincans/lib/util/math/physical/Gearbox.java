package org.austincans.lib.util.math.physical;

/** Translator that converts motor output to mechanism output */
public class Gearbox {
    private final double totalReduction;

    public Gearbox(GearStage... stages) {
        double reduction = 1.0;
        for (GearStage stage : stages) {
            reduction *= stage.getReduction();
        }
        totalReduction = reduction;
    }

    public double toMechanism(double motorValue) {
        return motorValue / totalReduction;
    }

    public double toMotor(double mechanismValue) {
        return mechanismValue * totalReduction;
    }

    /** Represents two meshed gears */
    public static record GearStage(int drivingGearTeeth, int drivenGearTeeth) {
        public double getReduction() {
            return (double) drivenGearTeeth / drivingGearTeeth;
        }
    }
}
