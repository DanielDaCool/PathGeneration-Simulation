// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.swing.plaf.basic.BasicBorders.RadioButtonBorder;

import com.pathplanner.lib.PathPlannerTrajectory.Waypoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.Constants;

/** Add your docs here. */
public class PathGenerator {

        PathPoint[] states;
        ArrayList<PathPoint> pathpoints;
        TrajectoryConfig config;

        public PathGenerator(TrajectoryConfig config, PathPoint... states) {
                this.states = states;
                this.config = config;
                pathpoints = new ArrayList<PathPoint>();
        }

        public Trajectory getOriginalPoints() {
                ArrayList<PathPoint> original = new ArrayList<PathPoint>();
                for (PathPoint point : states) {
                        original.add(point);
                }
                return new Trajectory(PathPointToState(original));
        }

        public Trajectory generateTrajectory() {
                pathpoints = new ArrayList<PathPoint>();
                pathpoints.add(states[0]);
                for (int i = 1; i < states.length - 1; i++) {
                        ArrayList<PathPoint> tempArr = transform(states[i - 1], states[i], states[i + 1]);

                        PathPoint[] arr = tempArr.toArray(new PathPoint[tempArr.size()]);
                        for (int j = 0; j < arr.length; j++) {
                                pathpoints.add(arr[j]);
                        }
                }

                pathpoints.add(states[states.length - 1]);
                return new Trajectory(PathPointToState(pathpoints));
        }

        public PathPoint[] generatePathPointArray() {
                PathPoint[] PPArr = new PathPoint[generateTrajectory().getStates().size()];
                int index = 0;
                PPArr[0] = calcFirstPoint(states[0].pose, states[1].pose, states[0].velocitySize);
                System.out.println(PPArr[0].velociotyTranslation.toString());
                index++;
                for (int i = 1; i < states.length - 1; i++) {
                        ArrayList<PathPoint> tempArr = transform(states[i - 1], states[i], states[i + 1]);

                        PathPoint[] arr = tempArr.toArray(new PathPoint[tempArr.size()]);
                        for (int j = 0; j < arr.length; j++) {
                                PPArr[index] = arr[j];
                                index++;
                        }
                }

                PPArr[index] = (states[states.length - 1]);
                return PPArr;
        }

        private ArrayList<PathPoint> transform(PathPoint prev, PathPoint cur, PathPoint next) {
                
                Pose2d previousPathPoint = prev.pose;
                Pose2d pathPoint = cur.pose;
                Pose2d nextPathPoints = next.pose;
                double radius = cur.radius > maxRadius(prev, cur, next) ? maxRadius(prev, cur, next) : cur.radius;
                cur.radius = radius;
                double velocityAtPathPoint = cur.velocitySize;

                Translation2d circle = calcCircleCenter(previousPathPoint, pathPoint, nextPathPoints, radius);
                // finding the line equasions, line - AX + BY + C -> [A,B,C]
                double[] firstLine = {
                                pathPoint.getY() - previousPathPoint.getY(),
                                previousPathPoint.getX() - pathPoint.getX(),
                                pathPoint.getX() * previousPathPoint.getY()
                                                - previousPathPoint.getX() * pathPoint.getY()
                };
                double[] secondLine = {
                                pathPoint.getY() - nextPathPoints.getY(),
                                nextPathPoints.getX() - pathPoint.getX(),
                                pathPoint.getX() * nextPathPoints.getY() - nextPathPoints.getX() * pathPoint.getY()
                };

                if (circle != null) {

                        /// find tangent points
                        Translation2d firstTangent = tangionPointOfLine(firstLine, circle);
                        Translation2d seconTangent = tangionPointOfLine(secondLine, circle);

                        // calculate the angle between tangent points on circle
                        Translation2d first = firstTangent.minus(circle);
                        Translation2d second = seconTangent.minus(circle);
                        double angleByArgPoints = second.getAngle().minus(first.getAngle()).getDegrees();

                        double increment = calculateIncrement(velocityAtPathPoint, radius);
                        // generate the list of points
                        ArrayList<PathPoint> points = new ArrayList<PathPoint>();
                        double holonomicRotation = pathPoint.getRotation().minus(previousPathPoint.getRotation())
                                        .getDegrees();
                        for (int i = 0; i < Math.abs(angleByArgPoints); i += increment) {
                                boolean isCircle = false;
                                Translation2d rotated = circle.plus(new Translation2d(radius, first.getAngle())
                                                .rotateBy(Rotation2d.fromDegrees(Math.signum(angleByArgPoints) * i)));
                                Translation2d velocity;
                                Translation2d firstToSecondPoint;
                                if (i + increment < Math.abs(angleByArgPoints)) {
                                        firstToSecondPoint = new Translation2d(radius, first.getAngle())
                                                        .rotateBy(Rotation2d.fromDegrees(Math.signum(angleByArgPoints)
                                                                        * (i + Math.signum(angleByArgPoints)
                                                                                        * increment)))
                                                        .minus(new Translation2d(radius, first.getAngle())
                                                                        .rotateBy(Rotation2d.fromDegrees(
                                                                                        Math.signum(angleByArgPoints)
                                                                                                        * i)))
                                                        .rotateBy(Rotation2d
                                                                        .fromDegrees(angleByArgPoints > 0 ? 0 : 180));
                                        velocity = new Translation2d(Math.min(velocityAtPathPoint, config.getMaxVelocity()) ,
                                                        firstToSecondPoint.getAngle());
                                        isCircle = true;
                                } else {
                                        velocity = new Translation2d(velocityAtPathPoint,
                                                        nextPathPoints.getTranslation().minus(rotated).getAngle());
                                }
                                points.add(new PathPoint(
                                                new Pose2d(rotated, previousPathPoint.getRotation()
                                                                .plus(Rotation2d.fromDegrees(holonomicRotation
                                                                                * (Math.abs(i) / angleByArgPoints)))),
                                                velocity, 0, isCircle));
                        }

                        return points;
                } else {
                        ArrayList<PathPoint> points = new ArrayList<PathPoint>();
                        Translation2d velocityTranslation2d = new Translation2d(velocityAtPathPoint,
                                        nextPathPoints.getTranslation().minus(pathPoint.getTranslation()).getAngle());
                        points.add(new PathPoint(pathPoint, velocityTranslation2d, 0, false));
                        System.out.println("NOT CIRCLE!!!");
                        System.out.println(points.get(0).pose.toString() + " VEL: " + points.get(0).velociotyTranslation.toString());
                        return points;
                }
        }

        //the calculatios for the first points on the path:
        private PathPoint calcFirstPoint(Pose2d first, Pose2d second, double velocityAtPoint){
                        Translation2d velocityTranslation2d = new Translation2d(velocityAtPoint,
                                        second.getTranslation().minus(first.getTranslation()).getAngle());
                        return new PathPoint(first, velocityTranslation2d, 0, false);

        }

        private ArrayList<State> PathPointToState(ArrayList<PathPoint> points) {
                ArrayList<State> result = new ArrayList<>();
                for (PathPoint iterable : points) {
                        State temp = new State();
                        temp.poseMeters = iterable.pose;
                        temp.velocityMetersPerSecond = iterable.velocitySize;
                        result.add(temp);
                }
                return result;
        }

        private Translation2d calcCircleCenter(Pose2d previousPose, Pose2d pose, Pose2d nextPose, double radius) {
                double x1 = previousPose.getX(), x2 = pose.getX(), x3 = nextPose.getX();
                double y1 = previousPose.getY(), y2 = pose.getY(), y3 = nextPose.getY();
                double k = (y2 * x3) - (y1 * x3) + (y1 * x2) - (y3 * x2) + (y3 * x1) - (y2 * x1);
                if (k == 0) {
                        return null;
                }
                double d1 = Math.signum((x2 - x1) * y3 - (y2 - y1) * x3 - y1 * x2 + y2 * x1)
                                * Math.sqrt((Math.pow((x3 - x2), 2) + Math.pow((y3 - y2), 2)));
                double d2 = Math.signum((x3 - x2) * y1 - (y3 - y2) * x1 - y2 * x3 + y3 * x2)
                                * Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
                double x = x2 + (radius * ((x2 - x1) * d1 + (x2 - x3) * d2)) / k;
                double y = y2 + (radius * ((y2 - y1) * d1 + (y2 - y3) * d2)) / k;

                return new Translation2d(x, y);
        }

        private double calculateIncrement(double velocityAtPoint, double radius) {
                double d = velocityAtPoint / Constants.CYCLES_PER_SECOND;
                return Math.max(Math.toDegrees(Math.acos(((d * d) - 2 * radius * radius) / (-2 * radius * radius))), 5);
        }

        private Translation2d tangionPointOfLine(double[] line, Translation2d circle) {
                double a = line[0], b = line[1], c = line[2];
                double x = (b * b * circle.getX() - a * b * circle.getY() - a * c)
                                / (a * a + b * b);
                double y = -(b * c + a * b * circle.getX() - a * a * circle.getY())
                                / (a * a + b * b);
                Translation2d res = new Translation2d(x, y);
                return res;
        }

        private double maxRadius(PathPoint prev, PathPoint cur, PathPoint next){
                double prevToCurDistance = Math.abs(cur.pose.getTranslation().minus(prev.pose.getTranslation()).getNorm());
                double curToNextDistance = Math.abs(next.pose.getTranslation().minus(cur.pose.getTranslation()).getNorm());
                double maxRadius = Math.abs(prevToCurDistance) - prev.radius;
                if(maxRadius > curToNextDistance){
                        maxRadius = curToNextDistance;
                }
                System.out.println("PREVTCUR: " + prevToCurDistance + " CURTNECT: " + curToNextDistance + " MAX: " + maxRadius);
                return maxRadius;
        }

}
