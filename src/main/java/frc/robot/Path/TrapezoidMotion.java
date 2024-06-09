// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path;

import java.util.function.Supplier;

/** Add your docs here. */
public class TrapezoidMotion {
    double distance;
    double endVelocityMS; // METER/SECOND
    double maxVelocityMC; // METER/CYCLE
    double acceleration;
    Supplier<Double> distanceMoved;
    Supplier<Double> currentVelocity;

    public TrapezoidMotion(double distance, double endCelocity, double maxVelocity, double acceleration,
            Supplier<Double> distanceMoved, Supplier<Double> currentVelocity) {
        this.distance = distance; // DISTANCE TO MOVE - MUST BE POSITIVE
        this.endVelocityMS = endCelocity;
        this.maxVelocityMC = maxVelocity;
        this.acceleration = acceleration;
        this.distanceMoved = distanceMoved; // DISTENCE MOVED - MUST BE POSITIVE, MUST BE IN RELATIVE TO THE START POINT
        this.currentVelocity = currentVelocity;
    }

    public double calculate() {
        double distanceLeft = distance - distanceMoved.get(); // MUST BE POSITIVE
        if (distanceLeft > distanceToEndVelDec()) {
            return Math.min(currentVelocity.get() + acceleration, maxVelocityMC);
        } else {
            return Math.max(endVelocityMS, currentVelocity.get() - acceleration);
        }
    }

    public double distanceToEndVelDec() {
        return (Math.pow(endVelocityMS, 2) - Math.pow(currentVelocity.get(), 2))
                / (2 * (acceleration * -1));
    }

    public boolean isFinished() {
        return distanceMoved.get() >= distance;
    }

}
