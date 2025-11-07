// Copyright (c) 2025 the ausTIN CANs
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
// Updated on Wednesday November 05, 2025 at 18:44:58 CST

package org.austincans.frc2158.util.gslam;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import org.austincans.frc2158.Constants;

public class GenericSlamIOSim implements GenericSlamIO {
    private final DCMotor gearbox;
    private final double maxTravel;
    private final Matrix<N2, N2> A;
    private final Vector<N2> B;

    protected Vector<N2> simState = VecBuilder.fill(0.0, 0.0);
    private double inputTorqueCurrent = 0.0;
    private double appliedVolts = 0.0;

    public GenericSlamIOSim(DCMotor gearbox, double moi, double maxTravel) {
        this.gearbox = gearbox;
        this.maxTravel = maxTravel;
        A = MatBuilder.fill(
                Nat.N2(), Nat.N2(), 0, 1, 0, -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * moi));
        B = VecBuilder.fill(0.0, gearbox.KtNMPerAmp / moi);
    }

    @Override
    public void updateInputs(GenericSlamIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        update(Constants.loopPeriodSecs);
        inputs.data = new GenericSlamIOData(
                true,
                simState.get(0),
                simState.get(1),
                appliedVolts,
                (appliedVolts / 12.0) * inputTorqueCurrent,
                inputTorqueCurrent,
                0.0);
    }

    @Override
    public void runTorqueCurrent(double amps) {
        inputTorqueCurrent = amps;
        appliedVolts = gearbox.getVoltage(gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0));
        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    }

    @Override
    public void stop() {
        runTorqueCurrent(0.0);
    }

    private void update(double dt) {
        inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -40.0, 40.0);
        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> u) -> A.times(x).plus(B.times(u)),
                simState,
                VecBuilder.fill(inputTorqueCurrent * 15.0),
                dt);
        // Apply limits
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
        if (simState.get(0) <= 0.0) {
            simState.set(1, 0, 0.0);
            simState.set(0, 0, 0.0);
        }
        if (simState.get(0) >= maxTravel) {
            simState.set(1, 0, 0.0);
            simState.set(0, 0, maxTravel);
        }
    }
}
