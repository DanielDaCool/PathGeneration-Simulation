// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path;

import java.util.function.Supplier;

/** Add your docs here. */
public class TrapezoidMotion {
    double distance;
    double endVelocity;
    double maxVelocity;
    double acceleration;
    Supplier<Double> distanceMoved;
    Supplier<Double> currentVelocity;

    public TrapezoidMotion(double distance, double endCelocity, double maxVelocity, double acceleration
        , Supplier<Double> distanceMoved, Supplier<Double> currentVelocity){
        this.distance = distance;
        this.endVelocity = endCelocity;
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.distanceMoved = distanceMoved;
        this.currentVelocity = currentVelocity;
    }

    public double calculate(){
        double distanceLeft = distance - distanceMoved.get();
        System.out.println("DL: " + distanceLeft);
        System.out.println("DLV: " + distanceToEndVelDec());
        if(distanceLeft > distanceToEndVelDec()){
            System.out.println("ACCELERATE");
            return Math.min(currentVelocity.get() + acceleration, maxVelocity);
        }else{
            System.out.println("DEACCELERATE");
            return Math.max(endVelocity, currentVelocity.get() - acceleration);
        }
    }

    private double distanceToEndVelDec(){
        System.out.println("DTEV=== END " + endVelocity + " CUR: " + currentVelocity.get() + " ACCEL " + acceleration);
        return (Math.pow(endVelocity, 2) - Math.pow(currentVelocity.get(), 2))/(2*(acceleration*-1));
    }

    public boolean isFinished(){
        return distanceMoved.get() >= distance;
    }

}
