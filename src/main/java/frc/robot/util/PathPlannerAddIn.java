// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.Waypoint;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;

/** Add your docs here. */
public class PathPlannerAddIn extends PathPlanner {
    /**
     * Generate a path on-the-fly from a list of points As you can't see the path in
     * the GUI when
     * using this method, make sure you have a good idea of what works well and what
     * doesn't before
     * you use this method in competition. Points positioned in weird configurations
     * such as being too
     * close together can lead to really janky paths.
     *
     * @param constraints The max velocity and max acceleration of the path
     * @param points      Points in the path
     * @param initSpeed   the initial speed of the robot
     * @return The generated path
     */
    public static PathPlannerTrajectory generatePath(
            PathConstraints constraints, List<PathPoint> points, double initSpeed) {
        return generatePath(constraints, false, points, initSpeed);
    }

    public static PathPlannerTrajectory generatePath(
            PathConstraints constraints, boolean reversed, List<PathPoint> points, double initSpeed) {
        return generatePath(constraints, reversed, points, new ArrayList<>(), initSpeed);
    }

    public static PathPlannerTrajectory generatePath(
            PathConstraints constraints,
            boolean reversed,
            List<PathPoint> points,
            List<EventMarker> eventMarkers,
            double initSpeed) {
        if (points.size() < 2) {
            throw new IllegalArgumentException(
                    "Error generating trajectory.  List of points in trajectory must have at least two points.");
        }

        PathPoint firstPoint = points.get(0);

        List<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(
                new Waypoint(
                        firstPoint.position,
                        null,
                        null,
                        initSpeed,
                        firstPoint.holonomicRotation,
                        false,
                        false,
                        new PathPlannerTrajectory.StopEvent()));

        for (int i = 1; i < points.size(); i++) {
            PathPoint p1 = points.get(i - 1);
            PathPoint p2 = points.get(i);

            double thirdDistance = p1.position.getDistance(p2.position) / 3.0;

            double p1NextDistance = p1.nextControlLength <= 0 ? thirdDistance : p1.nextControlLength;
            double p2PrevDistance = p2.prevControlLength <= 0 ? thirdDistance : p2.prevControlLength;

            Translation2d p1Next = p1.position.plus(
                    new Translation2d(
                            p1.heading.getCos() * p1NextDistance, p1.heading.getSin() * p1NextDistance));
            waypoints.get(i - 1).nextControl = p1Next;

            Translation2d p2Prev = p2.position.minus(
                    new Translation2d(
                            p2.heading.getCos() * p2PrevDistance, p2.heading.getSin() * p2PrevDistance));
            waypoints.add(
                    new Waypoint(
                            p2.position,
                            p2Prev,
                            null,
                            p2.velocityOverride,
                            p2.holonomicRotation,
                            false,
                            false,
                            new PathPlannerTrajectory.StopEvent()));
        }

        return new PathPlannerTrajectory(waypoints, eventMarkers, constraints, reversed, false);
    }
}
