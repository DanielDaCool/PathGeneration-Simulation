// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Path.FollowPath;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.Path.PathGenerator;
import frc.robot.Path.PathPoint;
import frc.robot.commands.Drive;
import frc.robot.subsystems.chassis.Chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  Field2d field;
  PathGenerator pathgen;
  Trajectory traj;
  Trajectory origin;
  Chassis chassis;
  PathPoint oneP;
  PathPoint twoP;
  PathPoint threeP;
  PathPoint fourP;
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    field = new Field2d();
    Chassis.CreateInstance();
    chassis = Chassis.GetInstance();
    chassis.setDefaultCommand(new Drive(chassis, new XboxController(0)));
    SmartDashboard.putData(chassis);
    TrajectoryConfig config = new TrajectoryConfig(6, 4);
    Pose2d one = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(45));
    Pose2d two = new Pose2d(new Translation2d(2, 4), Rotation2d.fromDegrees(90));
    Pose2d three = new Pose2d(new Translation2d(4, 0), Rotation2d.fromDegrees(91));
    Pose2d four = new Pose2d(new Translation2d(6, 1), Rotation2d.fromDegrees(180));
    Pose2d five = new Pose2d(new Translation2d(14, 3), Rotation2d.fromDegrees(0));
    Pose2d six = new Pose2d(new Translation2d(3, 6.8), Rotation2d.fromDegrees(0));
    Pose2d seven = new Pose2d(new Translation2d(9.4, 8), Rotation2d.fromDegrees(45));
    Pose2d eight = new Pose2d(new Translation2d(0, 8), Rotation2d.fromDegrees(-270));
    Pose2d nine = new Pose2d(new Translation2d(0.65, 2), Rotation2d.fromDegrees(-270));

    oneP = new PathPoint(one, 1, 0);
    twoP = new PathPoint(two, 1, 0.5);
    threeP = new PathPoint(three, 1, 0.5);
    fourP = new PathPoint(four, 1, 0.4);
    PathPoint fiveP = new PathPoint(five, 6, 0.3);
    PathPoint sixP = new PathPoint(six, 6, 0.4);
    PathPoint sevenP = new PathPoint(seven, 3, 0.4);
    PathPoint eightP = new PathPoint(eight, 1, 0.5);
    PathPoint nineP = new PathPoint(nine, 0, 0);

    SmartDashboard.putData(field);

    pathgen = new PathGenerator(config, oneP, twoP, threeP, fourP, fiveP, sixP, sevenP, eightP, nineP);
    traj = pathgen.generateTrajectory();
    origin = pathgen.getOriginalPoints();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TrajectoryConfig config = new TrajectoryConfig(6, 4);
    // return new InstantCommand(() -> {
    // field.getObject("path").setTrajectory(traj);
    // field.getObject("OG").setTrajectory(origin);
    // }).andThen(
    // new FollowPath(pathgen.generatePathPointArray(),
    // config, chassis, chassis::setVelocities, chassis::getPose,
    // chassis::getVelocity, chassis::getAngularVelocity));
    return new RunCommand(() -> {
      chassis.setVelocities(0, 0, 2);
    }, chassis);
  }
}