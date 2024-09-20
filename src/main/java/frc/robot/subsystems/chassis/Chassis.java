// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import java.util.Arrays;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.commands.chassis.PPSwerveControl;
import frc.robot.subsystems.chassis.ChassisConstants.SwerveModuleConstants;
import frc.robot.subsystems.chassis.utils.SwerveModule;
import frc.robot.subsystems.chassis.utils.simulation.SimulateGyro;
import frc.robot.utilities.UtilsGeneral;

/**
 * The subsystem that controls the robot's swerve chassis
 */
public class Chassis extends SubsystemBase {
    private final Field2d pathDisplay;

    public final Field2d field;
    private final SwerveModule[] modules;
    private final SimulateGyro gyro;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final PIDController angleController;
    private boolean isBreak;

    private Translation2d lastVel;

    static Chassis instance;

    public static Chassis CreateInstance() {
        if (instance == null) {
            instance = new Chassis();
        }
        return instance;
    }

    public static Chassis GetInstance() {
        return instance;
    }

    public void setPosToFrontNodeTest() {
        poseEstimator.resetPosition(getGyroRotation(), getModulePositions(),
                new Pose2d(new Translation2d(14, 0.5), Rotation2d.fromDegrees(0)));
    }

    /**
     * Creates a new Chassis.
     */
    private Chassis() {
        field = new Field2d();
        pathDisplay = new Field2d();
        // gyro = new PigeonIMU(14);
        gyro = new SimulateGyro();
        modules = new SwerveModule[] {
                new SwerveModule(SwerveModuleConstants.FRONT_LEFT),
                new SwerveModule(SwerveModuleConstants.FRONT_RIGHT),
                new SwerveModule(SwerveModuleConstants.BACK_LEFT),
                new SwerveModule(SwerveModuleConstants.BACK_RIGHT)
        };
        angleController = new PIDController(ChassisConstants.TELEOP_ROTATION_KP,
                ChassisConstants.TELEOP_ROTATION_KI, 0);
        angleController.enableContinuousInput(0, 2 * Math.PI);
        angleController.setTolerance(ChassisConstants.TELEOP_ANGLE_TOLERANCE);
        poseEstimator = new SwerveDrivePoseEstimator(ChassisConstants.KINEMATICS, getGyroRotation(),
                getModulePositions(), new Pose2d(0, 0, getGyroRotation()),
                VecBuilder.fill(Constants.LIMELIGHT_TRUST_VALUE, Constants.LIMELIGHT_TRUST_VALUE,
                        Constants.LIMELIGHT_TRUST_VALUE),
                VecBuilder.fill(Constants.ODOMETRY_TRUST_VALUE, Constants.ODOMETRY_TRUST_VALUE,
                        Constants.ODOMETRY_TRUST_VALUE));
        isBreak = true;

        setupPathDisplay();

        lastVel = new Translation2d();
    }

    private void setupPathDisplay() {
        // PPSwerveControl.setLoggingCallbacks((traj) -> {
        // pathDisplay.getObject("traj").setTrajectory(traj);
        // }, pathDisplay::setRobotPose, null, null);
    }

    /**
     * Gets the angle of the robot according to the gyro
     * 
     * @return The angle of the robot, between 0 and 360 degrees
     */
    public double getGyroAngle() {
        // return UtilsGeneral.normalizeDegrees(gyro.getFusedHeading());
        return gyro.GetAngle();
    }

    /**
     * Gets the rotation of the robot according to the gyro
     * 
     * @return The rotation of the robot
     */
    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(getGyroAngle());
    }

    /**
     * Gets the angle of the robot according to the pose estimator
     * 
     * @return The angle of the robot, between 0 and 360 degrees
     */
    public double getAngle() {
        return UtilsGeneral.normalizeDegrees(getRotation().getDegrees());
    }

    /**
     * Gets the rotation of the robot according to the pose estimator
     * 
     * @return The rotation of the robot
     */
    public Rotation2d getRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * Sets the velocities of the robot
     * 
     * @param vx    The x velocity, in meters per second
     * @param vy    The y velocity, in meters per second
     * @param omega The angular velocity, in radians per second
     */
    public void setVelocities(double vx, double vy, double omega) {
        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);
        SmartDashboard.putNumber("omega", omega);
        lastVel = new Translation2d(vx, vy);
        if (vx == 0 && vy == 0 && omega == 0 && getVelocity().getNorm() <= ChassisConstants.STOP_VELOCITY)
            stop();
        else {
            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getRotation());
            SwerveModuleState[] states = ChassisConstants.KINEMATICS.toSwerveModuleStates(speeds, getPose(),
                    getModuleStates());
            setModuleStates(states);
        }
    }

    public void setVelocities(ChassisSpeeds speeds) {
        setVelocities(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /**
     * Sets the velocities of the robot, but limits the acceleration
     * 
     * @param vx    The x velocity, in meters per second
     * @param vy    The y velocity, in meters per second
     * @param omega The angular velocity, in radians per second
     */
    public void setVelocitiesAcceleration(double vx, double vy, double omega) {
        Translation2d vel = UtilsGeneral
                .normalizeTranslation(new Translation2d(vx, vy).minus(lastVel),
                        ChassisConstants.MAX_ACCELERATION / Constants.CYCLES_PER_SECOND)
                .plus(lastVel);
        setVelocities(vel.getX(), vel.getY(), omega);
    }

    /**
     * Sets the velocities and the angle of the robot
     * 
     * @param vx    The x velocity, in meters per second
     * @param vy    The y velocity, in meters per second
     * @param angle The angle of the robot, in radians
     */
    public void setAngleAndVelocity(double vx, double vy, double angle) {
        angleController.setSetpoint(UtilsGeneral.normalizeRadians(angle));
        double omega = 0;
        if (!angleController.atSetpoint())
            omega = angleController.calculate(UtilsGeneral.normalizeRadians(getRotation().getRadians()));
        setVelocities(vx, vy, omega);
    }

    /**
     * Sets the velocities and the angle of the robot, but limits the acceleration
     * 
     * @param vx    The x velocity, in meters per second
     * @param vy    The y velocity, in meters per second
     * @param angle The angle of the robot, in radians
     */
    public void setAngleVelocityWithAcceleration(double vx, double vy, double angle) {
        Translation2d vel = UtilsGeneral
                .normalizeTranslation(new Translation2d(vx, vy).minus(lastVel),
                        ChassisConstants.MAX_ACCELERATION / Constants.CYCLES_PER_SECOND)
                .plus(lastVel);
        setAngleAndVelocity(vel.getX(), vel.getY(), angle);
    }

    /**
     * Sets the states of the modules
     * 
     * @param states The states of the modules, in order of front left, front right,
     *               back left, back right
     */
    public void setModuleStates(SwerveModuleState[] states) {

        SwerveDriveKinematics.desaturateWheelSpeeds(states, ChassisConstants.MAX_SPEED);
        for (int i = 0; i < 4; i++) {
            states[i] = SwerveModuleState.optimize(states[i], modules[i].getAngleRotation());
            modules[i].setState(states[i]);
        }

    }

    /**
     * Gets the states of the modules
     * 
     * @return The states of the modules, in order of front left, front right, back
     *         left, back right
     */
    private SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    /**
     * Gets the pose of the robot
     * 
     * @return The pose of the robot
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Stops all motors
     */
    public void stop() {
        Arrays.stream(modules).forEach(SwerveModule::stop);
    }

    public void setRampPosition() {
        Arrays.stream(modules).forEach((module) -> {
            module.setAngle(90 - getAngle());
            module.setVelocity(0);
        });
    }

    /**
     * Swaps the neutral mode of the modules, between brake and coast
     */
    public void swapNeutralMode() {
        isBreak = !isBreak;
        Arrays.stream(modules).forEach((module) -> module.setNeutralMode(isBreak));
    }

    public void setCoast() {
        Arrays.stream(modules).forEach((module) -> module.setNeutralMode(false));
    }

    public void setBreak() {
        Arrays.stream(modules).forEach((module) -> module.setNeutralMode(true));
    }

    /**
     * Resets the angle of the robot, so the forward of the robot is the same as the
     * forward of the field
     */
    public void resetAngle() {
        // gyro.setYaw(0);
        // gyro.setFusedHeading(0);
        Rotation2d rotation = getGyroRotation();
        poseEstimator.resetPosition(getGyroRotation(), getModulePositions(),
                new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), rotation));
    }

    public void setAngleTo180DependsOnAlliance() {
        double angle = DriverStation.getAlliance() == Alliance.Blue ? 180 : 0;
        // double angle = !UtilsGeneral.isRedAlliance() ? 180 : 0;
        System.out.println(angle);
        // gyro.setYaw(0);
        // gyro.setFusedHeading(0);
        Rotation2d rotation = getGyroRotation();
        poseEstimator.resetPosition(rotation, getModulePositions(),
                new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(),
                        rotation.rotateBy(Rotation2d.fromDegrees(angle))));
    }

    public void setAngleTo0DependsOnAlliance() {
        double angle = DriverStation.getAlliance() == Alliance.Blue ? 0 : 180;
        // double angle = !UtilsGeneral.isRedAlliance()? 0 : 180;
        System.out.println(angle);
        // gyro.setYaw(0);
        // gyro.setFusedHeading(0);
        Rotation2d rotation = getGyroRotation();
        poseEstimator.resetPosition(rotation, getModulePositions(),
                new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(),
                        rotation.rotateBy(Rotation2d.fromDegrees(angle))));
    }

    /**
     * Gets the positions of the modules
     * 
     * @return The positions of the modules, in order of front left, front right,
     *         back left, back right
     */
    private SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    /**
     * Gets the velocity of the robot
     * 
     * @return The velocity of the robot, in meters per second
     */
    public Translation2d getVelocity() {
        ChassisSpeeds speeds = ChassisConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(getRotation());
    }

    public Rotation2d getAngularVelocity() {
        ChassisSpeeds speeds = ChassisConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
        return Rotation2d.fromRadians(speeds.omegaRadiansPerSecond);
    }

    /**
     * Gets the roll of the robot
     * 
     * @return The roll of the robot
     */
    public double getRoll() {
        return 0;
    }

    /**
     * Gets the pitch of the robot
     * 
     * @return The pitch of the robot
     */
    public double getPitch() {
        return 0;
    }

    /**
     * Gets the upwards rotation of the robot
     * 
     * @return The upwards rotation of the robot
     */
    public double getUpRotation() {
        double pitch = getPitch();
        double roll = getRoll();
        double sign;
        if (Math.abs(pitch) > Math.abs(roll))
            sign = Math.signum(pitch);
        else
            sign = Math.signum(roll);
        return sign * Math.hypot(pitch, roll);
    }

    /**
     * Gets the upwards angular velocity of the robot
     * 
     * @return The upwards angular velocity of the robot
     */
    public double getUpAngularVel() {
        double[] arr = new double[3];
        // gyro.getRawGyro(arr);
        double sign;
        if (Math.abs(arr[0]) > Math.abs(arr[1]))
            sign = Math.signum(arr[0]);
        else
            sign = Math.signum(arr[1]);
        return sign * Math.hypot(arr[0], arr[1]);
    }

    /**
     * Represents a module on the robot
     */
    public enum Module {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT;

        /**
         * Gets the index of the module
         * 
         * @return The index of the module
         */
        public int getIndex() {
            return ordinal();
        }
    }

    /**
     * Sets up the vision listener, so that {@link #addVisionInput(Pair)} can be
     * used when vision data is received
     */

    @Override
    public void periodic() {
        gyro.setSpeedDS(Math.toDegrees(ChassisConstants.KINEMATICS
                .toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond));

        poseEstimator.update(getGyroRotation(), getModulePositions());
        field.setRobotPose(getPose());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putData("Front Left Module", modules[0]);
        SmartDashboard.putData("Front Right Module", modules[1]);
        SmartDashboard.putData("Back Left Module", modules[2]);
        SmartDashboard.putData("Back Right Module", modules[3]);

        SmartDashboard.putData("Field", field);

        SmartDashboard.putData("Trajectory", pathDisplay);

        UtilsGeneral.addDoubleProperty(builder, "Angle", this::getAngle, 2);

        UtilsGeneral.addDoubleProperty(builder, "UpAngle", this::getUpRotation, 2);
        UtilsGeneral.addDoubleProperty(builder, "UpAngularVel", this::getUpAngularVel, 2);

        // UtilsGeneral.putData("Change Neutral", "Change",
        // new InstantCommand(this::swapNeutralMode).ignoringDisable(true));

        UtilsGeneral.putData("setCoast", "setCoast",
                new InstantCommand(this::setCoast).ignoringDisable(true));

        UtilsGeneral.putData("setBreak", "setBreak",
                new InstantCommand(this::setBreak).ignoringDisable(true));

        UtilsGeneral.putData("Zero Angle", "Zero", new InstantCommand(this::resetAngle).ignoringDisable(true));

        UtilsGeneral.putData("Calibrate Offsets", "Calibrate", new InstantCommand(() -> {
            for (var module : modules) {
                module.calibrateOffset();
            }
        }).ignoringDisable(true));

        UtilsGeneral.addDoubleProperty(builder, "Velocity", () -> {
            return getVelocity().getNorm();
        }, 2);
        UtilsGeneral.addDoubleProperty(builder, "Velocity Angle", () -> {
            return getVelocity().getAngle().getDegrees();
        }, 2);

        builder.addBooleanProperty("Is Break", () -> isBreak, null);
    }
}
