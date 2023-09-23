// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class PathPoint {
    public Pose2d pose;
    public Translation2d velocity;
    public double radius;

    public PathPoint(Pose2d pose, Translation2d velocity, double radius){
        this.pose = pose;
        this.velocity = velocity;
        this.radius = radius;
    }

}
