package org.austincans.lib.subsystems;

import org.austincans.lib.io.motors.MotorIO;
import org.austincans.lib.io.motors.MotorIO.MotorIOOutputs;
import edu.wpi.first.math.Pair;

public abstract class BaseSubsystem {
    @SafeVarargs

    // TODO: 5. Need to make this class better

    public BaseSubsystem(Pair<MotorIO<?>, MotorIOOutputs>... pairs) {
        for (var pair : pairs) {
            pair.getFirst().initializeOutputs(pair.getSecond());
        }
    }
}
