// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Path.FollowPath;
import frc.robot.Path.FollowSection;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.Path.PathGenerator;
import frc.robot.Path.PathPoint;
import frc.robot.commands.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.XboxController;
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
    TrajectoryConfig config = new TrajectoryConfig(2, 2);
    Pose2d one = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    Pose2d two = new Pose2d(new Translation2d(2, 0), Rotation2d.fromDegrees(0));
    Pose2d three = new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(0));
    Pose2d four = new Pose2d(new Translation2d(5, 3), Rotation2d.fromDegrees(0));
    Pose2d five = new Pose2d(new Translation2d(7, 6.5), Rotation2d.fromDegrees(0));
    Pose2d six = new Pose2d(new Translation2d(13,6.8), Rotation2d.fromDegrees(0));
    Pose2d seven = new Pose2d(new Translation2d(9.4, 8), Rotation2d.fromDegrees(0));
    Pose2d eight = new Pose2d(new Translation2d(0, 8), Rotation2d.fromDegrees(0));
    Pose2d nine = new Pose2d(new Translation2d(1, 2), Rotation2d.fromDegrees(0));


    



     oneP = new PathPoint(one, new Translation2d(0,0), 0);
     twoP = new PathPoint(two, 3, 1);
     threeP = new PathPoint(three,  3, 1);
     fourP = new PathPoint(four, 3, 0.5);
    PathPoint fiveP = new PathPoint(five, 3, 0.5);
    PathPoint sixP = new PathPoint(six, 3, 0.5);
    PathPoint sevenP = new PathPoint(seven, 3, 0.5);
    PathPoint eightP = new PathPoint(eight, 3, 0.3);
    PathPoint nineP = new PathPoint(nine, new Translation2d(0,0), 0);



    SmartDashboard.putData(field);

    pathgen = new PathGenerator(config, oneP, twoP, threeP, fourP, fiveP, sixP, sevenP,eightP,nineP);
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
    }).andThen(new FollowPath(pathgen.generatePathPointArray(), 8, 12/50., chassis, chassis::setVelocities, chassis::getPose,
    chassis::getVelocity));
  }
}
