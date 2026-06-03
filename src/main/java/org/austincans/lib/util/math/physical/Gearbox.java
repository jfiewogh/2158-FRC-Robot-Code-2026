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

    public double inputToOutput(double input) {
        return input / totalReduction;
    }

    public double outputToInput(double output) {
        return output * totalReduction;
    }

    public double getReduction() {
        return totalReduction;
    }

    /** Represents two meshed gears */
    public static record GearStage(int drivingGearTeeth, int drivenGearTeeth) {
        /** Positive value means increase in torque and decrease in speed */
        public double getReduction() {
            return (double) drivenGearTeeth / drivingGearTeeth;
        }
    }
}
