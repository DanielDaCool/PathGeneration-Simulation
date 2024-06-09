// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
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

        /**
         * This function will take 3 sequential path point, and return an ArrayList
         * containing the transformed
         * middle point.
         * Meaning, if the middle point of the 3 argument points is a circle, the
         * ArrayList will contain the points
         * around the circle`s circumference.
         * If the middle point is not a circle, the ArrayList will contain the original
         * middle point with the
         * velocity vector pointing towards the next point.
         * 
         * @param prev the previus path point
         * @param cur  the current path point
         * @param next the next path point
         * @return ArrayList<PathPoint> an ArrayList containing the generated points.
         */
        private ArrayList<PathPoint> transform(PathPoint prev, PathPoint cur, PathPoint next) {

                Pose2d previousPathPoint = prev.pose;
                Pose2d pathPoint = cur.pose;
                Pose2d nextPathPoint = next.pose;

                // calculates 180 - the angle between the vector (prev->current) and
                // (current->next)
                double angleBetweenPoints = 180 - Math.abs(pathPoint.getTranslation()
                                .minus(previousPathPoint.getTranslation()).getAngle()
                                .minus(nextPathPoint.getTranslation().minus(pathPoint.getTranslation()).getAngle())
                                .getDegrees());

                // Determening the raduis at the point under the max radius limit
                double radius = 0;
                if (cur.radius != 0) {
                        if (cur.radius > maxRadius(prev, cur, next, angleBetweenPoints)) {
                                radius = maxRadius(prev, cur, next, angleBetweenPoints);
                        } else {
                                radius = cur.radius;
                        }
                }
                cur.radius = radius;

                // finding the equasions of the lines firstLine:(prev->curr),
                // secondLine:(curr->next)
                // , line: - AX + BY + C -> [A,B,C]
                double[] firstLine = {
                                pathPoint.getY() - previousPathPoint.getY(),
                                previousPathPoint.getX() - pathPoint.getX(),
                                pathPoint.getX() * previousPathPoint.getY()
                                                - previousPathPoint.getX() * pathPoint.getY()
                };
                double[] secondLine = {
                                pathPoint.getY() - nextPathPoint.getY(),
                                nextPathPoint.getX() - pathPoint.getX(),
                                pathPoint.getX() * nextPathPoint.getY() - nextPathPoint.getX() * pathPoint.getY()
                };

                Translation2d circle = calcCircleCenter(previousPathPoint, pathPoint, nextPathPoint, radius);
                double velocityAtPathPoint = cur.velocitySize;
                if (circle != null && radius != 0) {

                        /// find tangent points
                        Translation2d firstTangent = tangionPointOfLine(firstLine, circle);
                        Translation2d seconTangent = tangionPointOfLine(secondLine, circle);

                        // calculete the vectors from the circle center to the tangent points.
                        Translation2d circleCenterToFirstTan = firstTangent.minus(circle);
                        Translation2d circleCenterToSecondTan = seconTangent.minus(circle);

                        // calculate the crnter angle created by the to tangent points.
                        double canterAngle = circleCenterToSecondTan.getAngle().minus(circleCenterToFirstTan.getAngle())
                                        .getDegrees();

                        double increment = calculateIncrement(velocityAtPathPoint, radius);

                        // After calculating the circle, the increments, the angle created by the 3
                        // points, the increment,
                        // and the tangion points to the circle, we can generate the points on the
                        // circle`s circumference.
                        return generateTransformedPoints(previousPathPoint, pathPoint, nextPathPoint,
                                        circle, radius, increment, canterAngle, circleCenterToFirstTan,
                                        velocityAtPathPoint);

                } else {
                        // if there is no circle(meaning the raduis is 0, or thet the 3 points are on a
                        // straight line),
                        // we will return the current path point with it`s speed vector pointing to the
                        // next point.
                        ArrayList<PathPoint> points = new ArrayList<PathPoint>();
                        Translation2d velocityTranslation2d = new Translation2d(velocityAtPathPoint,
                                        nextPathPoint.getTranslation().minus(pathPoint.getTranslation()).getAngle());
                        points.add(new PathPoint(pathPoint, velocityTranslation2d, 0, false));
                        return points;
                }

        }

        /**
         * This function gets all the information required to construct and return an
         * ArrayList containing the
         * points that describe a circular path.
         * 
         * @param previousPathPoint
         * @param pathPoint
         * @param nextPathPoint
         * @param circle
         * @param radius
         * @param increment
         * @param angleByArgPoints
         * @param firstTanToCircle
         * @param velocityAtPathPoint
         * @return ArrayList<PathPoint> containig all the points around the circular
         *         path.
         */
        private ArrayList<PathPoint> generateTransformedPoints(Pose2d previousPathPoint, Pose2d pathPoint,
                        Pose2d nextPathPoint, Translation2d circle, double radius, double increment,
                        double canterAngle, Translation2d circleCenterToFirstTan, double velocityAtPathPoint) {

                // Initialize an empty ArrayList. the calculate points will be added to here.
                ArrayList<PathPoint> points = new ArrayList<PathPoint>();

                // The Holonomic roatation in degrees.
                double holonomicRotationAmount = pathPoint.getRotation().minus(previousPathPoint.getRotation())
                                .getDegrees();

                double angleOffset = 0;

                while (angleOffset < Math.abs(canterAngle)) {
                        boolean isCircle = false;

                        // calculates the point location on the circumference of the circle.
                        Translation2d rotated = circle.plus(new Translation2d(radius, circleCenterToFirstTan.getAngle())
                                        .rotateBy(Rotation2d.fromDegrees(Math.signum(canterAngle) * angleOffset)));

                        Translation2d velocity;
                        Translation2d firstToSecondPoint;
                        // checking if this is the last point on the circle
                        if (angleOffset + increment < Math.abs(canterAngle)) {
                                // if its not the last point on the circle:
                                Translation2d nextPointOnCircle = circle.plus(new Translation2d(radius,
                                                circleCenterToFirstTan.getAngle())
                                                .rotateBy(Rotation2d.fromDegrees(
                                                                Math.signum(canterAngle) * (angleOffset + increment))));

                                // calculating the vector from the current point to the next
                                firstToSecondPoint = nextPointOnCircle.minus(rotated);

                                // calculating the velocitry at current point
                                velocity = new Translation2d(Math.min(velocityAtPathPoint, config.getMaxVelocity()),
                                                firstToSecondPoint.getAngle());
                                isCircle = true;
                        } else {
                                // if its the last point on the circle
                                velocity = new Translation2d(Math.min(velocityAtPathPoint, config.getMaxVelocity()),
                                                nextPathPoint.getTranslation().minus(rotated).getAngle());
                        }
                        // add the new generated point to the list
                        points.add(new PathPoint(
                                        new Pose2d(rotated, previousPathPoint.getRotation().plus(
                                                        Rotation2d.fromDegrees(holonomicRotationAmount
                                                                        * (Math.abs(angleOffset)
                                                                                        / Math.abs(canterAngle))))),
                                        velocity, 0, isCircle));

                        if (angleOffset + increment < Math.abs(canterAngle)) {
                                angleOffset += increment;
                        } else {
                                angleOffset = Math.abs(canterAngle);
                        }

                }

                return points;
        }

        // the calculatios for the first points on the path:
        private PathPoint calcFirstPoint(Pose2d first, Pose2d second, double velocityAtPoint) {
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

        private double maxRadius(PathPoint prev, PathPoint cur, PathPoint next, double angleOfPoints) {
                double prevToCurDistance = Math.abs(
                                cur.pose.getTranslation().minus(prev.pose.getTranslation()).getNorm() - prev.radius);
                double curToNextDistance = Math
                                .abs(next.pose.getTranslation().minus(cur.pose.getTranslation()).getNorm());
                double shortLine = Math.min(prevToCurDistance, curToNextDistance);
                double maxRadius = Math.abs(Math.tan(Math.toRadians(angleOfPoints / 2)) * shortLine);
                System.out.println("PRV: " + prevToCurDistance + " NXT: " + curToNextDistance + " MIN: " + shortLine
                                + " MAX: " + maxRadius + " PRvRad " + prev.radius + " ANGLE: " + angleOfPoints);
                return maxRadius;
        }

}
