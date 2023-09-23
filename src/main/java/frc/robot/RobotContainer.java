// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Path.PathGenerator;
import frc.robot.Path.PathPoint;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    field = new Field2d();
    TrajectoryConfig config = new TrajectoryConfig(2, 2);
    Pose2d one = new Pose2d(new Translation2d(0, 2), Rotation2d.fromDegrees(0));
    Pose2d two = new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(0));
    Pose2d three = new Pose2d(new Translation2d(2, 4), Rotation2d.fromDegrees(0));
    Pose2d four = new Pose2d(new Translation2d(2.7, 4.5), Rotation2d.fromDegrees(0));
    Pose2d five = new Pose2d(new Translation2d(4, 2), Rotation2d.fromDegrees(0));
    Pose2d six = new Pose2d(new Translation2d(5, 4), Rotation2d.fromDegrees(0));
    Pose2d seven = new Pose2d(new Translation2d(5, 5), Rotation2d.fromDegrees(0));


    PathPoint oneP = new PathPoint(one, new Translation2d(), 0);
    PathPoint twoP = new PathPoint(two, new Translation2d(), 1);
    PathPoint threeP = new PathPoint(three, new Translation2d(), 0.5);
    PathPoint fourP = new PathPoint(four, new Translation2d(), 0.2);
    PathPoint fiveP = new PathPoint(five, new Translation2d(), 0.2);
    PathPoint sixP = new PathPoint(six, new Translation2d(), 0.2);
    PathPoint sevenP = new PathPoint(seven, new Translation2d(), 0);


    SmartDashboard.putData(field);

    pathgen = new PathGenerator(config, oneP, twoP, threeP, fourP, fiveP, sixP, sevenP);
    traj = pathgen.generateTrajectory();
    origin = pathgen.getOriginalPoints();
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand(() -> {
      field.getObject("path").setTrajectory(traj);
      field.getObject("OG").setTrajectory(origin);
    });
  }
}
