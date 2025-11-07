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

package org.austincans.frc2158.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class SparkUtil {
    /** Stores whether any error was has been detected by other utility methods. */
    public static boolean sparkStickyFault = false;

    /** Processes a value from a Spark only if the value is valid. */
    public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            consumer.accept(value);
        } else {
            sparkStickyFault = true;
        }
    }

    /** Processes a value from a Spark only if the value is valid. */
    public static void ifOk(SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
        double[] values = new double[suppliers.length];
        for (int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumer.accept(values);
    }

    /** Return a value from a Spark (or the default if the value is invalid). */
    public static double ifOkOrDefault(SparkBase spark, DoubleSupplier supplier, double defaultValue) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            return value;
        } else {
            sparkStickyFault = true;
            return defaultValue;
        }
    }

    /** Return a processed set of values from a Spark (or the default if one of the values is invalid). */
    public static double ifOkOrDefault(
            SparkBase spark, DoubleSupplier[] suppliers, Function<Double[], Double> transformer, double defaultValue) {
        Double[] values = new Double[suppliers.length];
        for (int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return defaultValue;
            }
        }
        return transformer.apply(values);
    }

    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error == REVLibError.kOk) {
                break;
            } else {
                sparkStickyFault = true;
            }
        }
    }
}
