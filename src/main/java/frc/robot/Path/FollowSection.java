// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class FollowSection extends CommandBase {
  /** Creates a new FollowSection. */
  PathPoint first;
  PathPoint second;
  TrapezoidMotion moveTrapezoid;
  TrapezoidMotion rotateTrapzoid;

  Subsystem subsystem;
  double maxSpeed;
  double maxAcceleration;
  Consumer<ChassisSpeeds> setSpeeds;
  Supplier<Pose2d> getPose;
  Supplier<Translation2d> getVelocity;
  PIDController positionPid;
  PIDController rotationPid;
  double[] path;
  Supplier<Rotation2d> angularVelocity;
  double AngleDistance;

  public FollowSection(PathPoint first, PathPoint second, TrajectoryConfig config,
      Subsystem subsystem, Consumer<ChassisSpeeds> setSpeeds, Supplier<Pose2d> getPose,
      Supplier<Translation2d> getVelocity, Supplier<Rotation2d> angularVelocity,
      PIDController positionPid, PIDController rotationPid) {
    this.first = first;
    this.second = second;

    this.subsystem = subsystem;
    maxSpeed = config.getMaxVelocity();
    maxAcceleration = config.getMaxAcceleration();
    this.setSpeeds = setSpeeds;
    this.getPose = getPose;
    this.angularVelocity = angularVelocity;
    this.getVelocity = getVelocity;
    this.positionPid = positionPid;
    this.rotationPid = rotationPid;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // find the algebric reprezentation of the line of the followed path
    Translation2d firstTranslation = first.pose.getTranslation();
    Translation2d secondTranslation = second.pose.getTranslation();
    path = new double[3];
    path[0] = secondTranslation.getY() - firstTranslation.getY();
    path[1] = firstTranslation.getX() - secondTranslation.getX();
    path[2] = secondTranslation.getX() * firstTranslation.getY() - firstTranslation.getX() * secondTranslation.getY();

    moveTrapezoid = new TrapezoidMotion(second.pose.getTranslation().minus(first.pose.getTranslation()).getNorm(),
        second.velocitySize, maxSpeed, maxAcceleration,
        () -> {
          return getPose.get().getTranslation().minus(first.pose.getTranslation()).rotateBy(
              Rotation2d.fromDegrees(-first.velociotyTranslation.getAngle().getDegrees())).getNorm();
        }, () -> {
          return getVelocity.get().getNorm();
        });
    AngleDistance = getAngleDistanceForTrapezoid(
        second.pose.getRotation().getDegrees(), getPose.get().getRotation().getDegrees());

    rotateTrapzoid = new TrapezoidMotion(getPose.get().getRotation().getDegrees() + Math.abs(AngleDistance), 0,
        360, 720 * 0.02,
        () -> {
          return Math.abs(getPose.get().getRotation().getDegrees());
        },
        () -> {
          return Math.abs(angularVelocity.get().getDegrees());
        });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d currentPose = getPose.get().getTranslation();

    // calculate the on path velocity vector
    Translation2d velocityOnPath;
    if (!first.isCircle || !second.isCircle) {
      double velocityOnPathSize = moveTrapezoid.calculate();
      velocityOnPath = new Translation2d(velocityOnPathSize, first.velociotyTranslation.getAngle());
    } else {
      velocityOnPath = first.velociotyTranslation;
    }

    // calculate the perpendicular to path velocity vector:
    // calculate the path representing vector
    Translation2d pathVector = second.pose.getTranslation().minus(first.pose.getTranslation());
    // calculate the robot`s perpendicular distance to the path:
    double perpendicularDistance = currentPose.minus(first.pose.getTranslation())
        .rotateBy(new Rotation2d(pathVector.getX(), -pathVector.getY())).getY();
    // use the PID to calculate the correction(perpendicular to the path velocity)
    // velocioty size
    double correctionSize = Math.abs(positionPid.calculate(perpendicularDistance));
    System.out.println("perp dis  " + perpendicularDistance + " perpvel  " + correctionSize + " dir " + 90
        * Math.signum(-perpendicularDistance));
    // calculate the correction speed vector
    Translation2d correctionVector = new Translation2d(correctionSize,
        first.velociotyTranslation.rotateBy(Rotation2d.fromDegrees(90 * Math.signum(-perpendicularDistance)))
            .getAngle());

    Translation2d combinedVelocity = velocityOnPath.plus(correctionVector);

    // calculate rotation
    // calculate PID correction
    double rotTarget = recalculateAngle(getPose.get().getRotation().getDegrees(),
        second.pose.getRotation().getDegrees());
    rotationPid.setSetpoint(rotTarget);
    // Add angle correction
    // angularVelocity +=
    // -rotPid.calculate(getPose.get().getRotation().getDegrees());

    // set the calculated speeds.
    setSpeeds
        .accept(
            new ChassisSpeeds(combinedVelocity.getX(), combinedVelocity.getY(), Math.toRadians(rotationPid.calculate(
                getPose.get().getRotation().getDegrees()))));
  }

  private double recalculateAngle(double currentAngle, double spAngle) {
    double res = spAngle;
    if (Math.abs(spAngle - currentAngle) > 180)
      res = res - 360 * Math.signum(res);
    return res;
  }

  private double getAngleDistanceForTrapezoid(double wantedAngle, double currentAngle) {
    double distance = wantedAngle - currentAngle;
    if (Math.abs(distance) > 180) {
      distance += Math.signum(-distance) * 360;
    }
    System.out.println("CALC A: " + distance);
    return distance;
  }

  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return moveTrapezoid.isFinished();
  }
}
