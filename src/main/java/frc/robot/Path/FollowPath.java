// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class FollowPath extends CommandBase {
  /** Creates a new FollowPath. */
  PathPoint[] pathPoints;
  TrapezoidMotion trapezoid;
  Subsystem subsystem;
  double maxSpeed;
  double maxAcceleration;
  Consumer<ChassisSpeeds> setSpeeds; 
  Supplier<Pose2d> getPose;
  Supplier<Translation2d> getVelocity;
  int currentSection = 0;
  FollowSection followSection;
  public FollowPath(PathPoint[] pathPoints, double maxSpeed, double maxAcceleration, 
    Subsystem subsystem, Consumer<ChassisSpeeds> setSpeeds, Supplier<Pose2d> getPose, Supplier<Translation2d> getVelocity) {
    this.pathPoints = pathPoints;
    this.subsystem = subsystem;
    this.maxSpeed = maxSpeed;
    this.maxAcceleration = maxAcceleration;
    this.setSpeeds = setSpeeds;
    this.getPose = getPose;
    this.getVelocity = getVelocity;
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateSection();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(followSection.isFinished())
      updateSection();
  }
  private void updateSection(){
    if(currentSection < pathPoints.length-1){
    followSection = new FollowSection(pathPoints[currentSection], pathPoints[currentSection + 1],
     maxSpeed, maxAcceleration, subsystem, setSpeeds, getPose, getVelocity, 2, 1, 0 , 1 , 0 , 0.0);
    followSection.schedule();
     currentSection++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
