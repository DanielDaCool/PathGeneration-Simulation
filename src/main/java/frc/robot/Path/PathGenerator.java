// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory.Waypoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;

/** Add your docs here. */
public class PathGenerator {

        PathPoint[] states;
        ArrayList<Pose2d> pathpoints;
        TrajectoryConfig config;

        public PathGenerator(TrajectoryConfig config, PathPoint... states) {
                this.states = states;
                this.config = config;
                pathpoints = new ArrayList<Pose2d>();
        }

        public Trajectory getOriginalPoints(){
                ArrayList<Pose2d> original = new ArrayList<Pose2d>();
                for(PathPoint point:states){
                        original.add(point.pose);
                }
                return new Trajectory(poseToState(original));
        }

        public Trajectory generateTrajectory() {
                
                

                pathpoints.add(new Pose2d(states[0].pose.getTranslation(), Rotation2d.fromDegrees(0)));
                System.out.println("STATES LENGTH" + states.length);
                for (int i = 1; i < states.length - 1; i++) {
                        ArrayList<Translation2d> tempArr = transform(states[i - 1].pose,
                                        states[i].pose, states[i + 1].pose, states[i].radius, 5);

                        Translation2d[] arr = tempArr.toArray(new Translation2d[tempArr.size()]);
                        System.out.println(arr.length + "LENGTHHH");
                        for (int j = 0; j < arr.length; j++) {
                                pathpoints.add(new Pose2d(arr[j], Rotation2d.fromDegrees(0)));
                                System.out.println("ADDED " + arr[j].toString());
                        }
                }


                pathpoints.add(new Pose2d(states[states.length - 1].pose.getTranslation(), Rotation2d.fromDegrees(0)));
                return new Trajectory(poseToState(pathpoints));
        }

        private ArrayList<Translation2d> transform(Pose2d previousPose, Pose2d pose, Pose2d nextPose, double radius,
                        double increment) {

                Translation2d circle = calcCircleCenter(previousPose, pose, nextPose, radius);
                System.out.println("test" + new Translation2d(1, 1).toString());
                System.out.println("CIRCLE::: " + circle.toString());
                // finding the line equasions, line - AX + BY + C -> [A,B,C]
                double[] firstLine = {
                                pose.getY() - previousPose.getY(),
                                previousPose.getX() - pose.getX(),
                                pose.getX() * previousPose.getY() - previousPose.getX() * pose.getY()
                };
                double[] secondLine = {
                                pose.getY() - nextPose.getY(),
                                nextPose.getX() - pose.getX(),
                                pose.getX() * nextPose.getY() - nextPose.getX() * pose.getY()
                };

                ///find tangent points
                Translation2d firstTangent = tangionPointOfLine(firstLine,circle);
                Translation2d seconTangent = tangionPointOfLine(secondLine,circle);

                // calculate the angle between tangent points on circle 
                Translation2d first = firstTangent.minus(circle);
                Translation2d second = seconTangent.minus(circle);
                double angleByArgPoints = second.getAngle().minus(first.getAngle()).getDegrees();
                System.out.println("ANGLE BETWEEN POINTS" + angleByArgPoints);

                // generate the list of points
                ArrayList<Translation2d> points = new ArrayList<Translation2d>();
                for (int i = 0; i < Math.abs(angleByArgPoints); i += increment) {
                        Translation2d rotated = circle.plus(new Translation2d(radius, first.getAngle())
                                .rotateBy(Rotation2d.fromDegrees(Math.signum(angleByArgPoints)*i)));

                        points.add(rotated);
                        System.out.println("ROTATED" + rotated.toString());
                }

                // calculate speeds
                // ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();
                // for (int i = 0; i < points.size(); i++) {
                //         // speeds
                // }

                // null!
                return points;
        }

        private ArrayList<State> poseToState(ArrayList<Pose2d> pose) {
                ArrayList<State> result = new ArrayList<>();
                for (Pose2d iterable : pose) {
                        State temp = new State();
                        temp.poseMeters = iterable;
                        temp.velocityMetersPerSecond = 1;
                        result.add(temp);
                }
                return result;
        }

        private Translation2d calcCircleCenter(Pose2d previousPose, Pose2d pose, Pose2d nextPose, double radius) {
                double x1 = previousPose.getX(), x2 = pose.getX(), x3 = nextPose.getX();
                double y1 = previousPose.getY(), y2 = pose.getY(), y3 = nextPose.getY();
                double k = (y2 * x3) - (y1 * x3) + (y1 * x2) - (y3 * x2) + (y3 * x1) - (y2 * x1);
                double d1 = Math.signum((x2-x1)*y3 - (y2-y1)*x3 -y1*x2 + y2*x1)
                        *Math.sqrt((Math.pow((x3-x2), 2) + Math.pow((y3 - y2), 2)));
                double d2 = Math.signum((x3-x2)*y1 - (y3-y2)*x1 - y2*x3 + y3*x2)
                        *Math.sqrt(Math.pow((x2-x1), 2) + Math.pow((y2 - y1), 2));
                double x = x2 + (radius*((x2-x1)*d1 + (x2-x3)*d2))/k;
                double y = y2 + (radius*((y2-y1)*d1 + (y2-y3)*d2))/k;
                System.out.println("x1, " + x1 + " y1 " + y1);
                System.out.println("x2, " + x2 + " y2 " + y2);
                System.out.println("x3, " + x3 + " y3 " + y3);
                System.out.println("d1 " + d1);
                System.out.println("d2 " + d2);
                System.out.println("k " + k);
                System.out.println("X " + x);
                System.out.println("Y " + y);


                return new Translation2d(x, y);
        }

        private Translation2d tangionPointOfLine(double[] line, Translation2d circle){
                double a = line[0], b = line[1], c = line[2];
                double x = (b*b*circle.getX() - a*b*circle.getY() - a*c)
                        /(a*a + b*b);
                double y = -(b*c + a*b*circle.getX() - a*a*circle.getY())
                        /(a*a + b*b);
                Translation2d res = new Translation2d(x,y);
                System.out.println("TAN POINT: " + res.toString());
                return res ;
        }

}
