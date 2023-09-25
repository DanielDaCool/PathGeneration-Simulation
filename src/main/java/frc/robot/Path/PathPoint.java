// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

/** Add your docs here. */
public class PathPoint {
    public Pose2d pose;
    public double velocitySize;
    public Translation2d velociotyTranslation;
    public double radius;
    public boolean isCircle;

    public PathPoint(Pose2d pose, double velocitySize, double radius){
        this.pose = pose;
        this.velocitySize = velocitySize;
        this.radius = radius;
    }

    public PathPoint(Pose2d pose, Translation2d velociotyTranslation, double radius){
        this.pose = pose;
        this.velociotyTranslation = velociotyTranslation;
        velocitySize = velociotyTranslation.getNorm();
        this.radius = radius;
    }

    public PathPoint(Pose2d pose, Translation2d velociotyTranslation, double radius, boolean isCircle){
        this(pose,velociotyTranslation,radius);
        this.isCircle = isCircle;
    }

}
