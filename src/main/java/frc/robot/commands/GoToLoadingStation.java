// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Path.FollowPath;
import frc.robot.Path.PathGenerator;
import frc.robot.Path.PathPoint;

import frc.robot.subsystems.chassis.Chassis;

public class GoToLoadingStation{
  /** Creates a new GoToLoadingStation. */
  
  
  static private PathPoint end = new PathPoint(new Pose2d(15, 8, Rotation2d.fromDegrees(0)),
    new Translation2d(0,0) , 0);
  static private PathPoint before = new PathPoint(new Pose2d(14, 8, Rotation2d.fromDegrees(0)),
    1.5 , 0.5);
  public GoToLoadingStation() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  static public Command go(Chassis chassis) {
    PathGenerator pathGen;
    PathPoint current = new PathPoint(chassis.getPose(), 4, 0.3);
    pathGen = new PathGenerator(new TrajectoryConfig(4, 10)
    , current, before, end);
    FollowPath follow = new FollowPath(pathGen.generatePathPointArray(),
    12, 10/50., chassis, chassis::setVelocities, chassis::getPose,
     chassis::getVelocity);
     chassis.field.getObject("path").setTrajectory(pathGen.generateTrajectory());
     return follow;
  }

}
