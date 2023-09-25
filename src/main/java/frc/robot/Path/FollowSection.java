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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.chassis.Chassis;

public class FollowSection extends CommandBase {
  /** Creates a new FollowSection. */
  PathPoint first;
  PathPoint second;
  TrapezoidMotion trapezoid;
  Subsystem subsystem;
  double maxSpeed;
  double maxAcceleration;
  Consumer<ChassisSpeeds> setSpeeds;
  Supplier<Pose2d> getPose;
  Supplier<Translation2d> getVelocity;
  PIDController posPid;
  PIDController rotPid;
  double[] line;

  public FollowSection(PathPoint first, PathPoint second, double maxSpeed, double maxAcceleration,
      Subsystem subsystem, Consumer<ChassisSpeeds> setSpeeds, Supplier<Pose2d> getPose,
      Supplier<Translation2d> getVelocity,
      double kp, double ki, double kd, double rotKp, double rotKi, double rotKd) {
    this.first = first;
    this.second = second;

    this.subsystem = subsystem;
    this.maxSpeed = maxSpeed;
    this.maxAcceleration = maxAcceleration;
    this.setSpeeds = setSpeeds;
    this.getPose = getPose;
    this.getVelocity = getVelocity;
    posPid = new PIDController(kp, ki, kd);
    rotPid = new PIDController(rotKp, rotKi, rotKd);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Translation2d firstTranslation = first.pose.getTranslation();
    Translation2d secondTranslation = second.pose.getTranslation();
    double[] tempLine = {
        secondTranslation.getY() - firstTranslation.getY(),
        firstTranslation.getX() - secondTranslation.getX(),
        secondTranslation.getX() * firstTranslation.getY() - firstTranslation.getX() * secondTranslation.getY()
    };
    line = tempLine;
    trapezoid = new TrapezoidMotion(second.pose.getTranslation().minus(first.pose.getTranslation()).getNorm(),
        second.velocitySize, maxSpeed, maxAcceleration,
        () -> {
          return getPose.get().getTranslation().minus(first.pose.getTranslation()).rotateBy(
              Rotation2d.fromDegrees(-first.velociotyTranslation.getAngle().getDegrees())).getNorm();
        }, () -> {
          return getVelocity.get().getNorm();
        });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d pose = getPose.get().getTranslation();
    Translation2d vec = second.pose.getTranslation().minus(first.pose.getTranslation());
    double error = -pose.minus(first.pose.getTranslation())
      .rotateBy(new Rotation2d(vec.getX(),-vec.getY())).getY();
    double correctionSize = Math.abs(posPid.calculate(error));
    Translation2d spvelTranslation;
    if (!first.isCircle || !second.isCircle) {
      double spvel = trapezoid.calculate();
      spvelTranslation = new Translation2d(spvel, first.velociotyTranslation.getAngle());
    } else {
      spvelTranslation = first.velociotyTranslation;
    }
    Translation2d correction = new Translation2d(correctionSize,
    first.velociotyTranslation.rotateBy(Rotation2d.fromDegrees(90 * Math.signum(error))).getAngle());
    Translation2d res = spvelTranslation.plus(correction);
    double rotTarget = normalaizeAngle(getPose.get().getRotation().getDegrees(), second.pose.getRotation().getDegrees());
    rotPid.setSetpoint(rotTarget);
    double angularVelocity = -rotPid.calculate(getPose.get().getRotation().getDegrees());

    System.out.println("T: " + rotTarget + " PPROT: " + second.pose.getRotation().getDegrees() + " VEL: " +  angularVelocity);
    setSpeeds.accept(new ChassisSpeeds(res.getX(), res.getY(), Math.toRadians(-angularVelocity)));
  }

  private double normalaizeAngle(double currentAngle, double spAngle){
    double res = spAngle;
    if(Math.abs(spAngle) - Math.abs(currentAngle) > 180)
      res = res - 360;
    return res;
  }

  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trapezoid.isFinished();
  }
}
