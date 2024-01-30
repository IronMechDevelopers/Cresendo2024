package frc.utils;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PathFactory {
    public static HashMap<Integer, Pose2d> locationMap;

    static {
        locationMap = new HashMap<>();
        locationMap.put(1, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
        locationMap.put(2, new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
    }

    /**
     * Create a path from one point to another
     * 
     * @param startPose a pose2d with the current robots location
     * @param endPose   a pose2D where we want the final robot location to be
     * @return the path to be followed
     */
    public static PathPlannerPath createPath(Pose2d startPose, Pose2d endPose, Rotation2d rotation ) {
        // Create a list of bezier points from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not
        // use holonomic rotation.
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                startPose, endPose);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a
                                                                         // differential drivetrain, the angular
                                                                         // constraints have no effect.
                new GoalEndState(0.0, new Rotation2d()) // Goal end state. You can set a holonomic rotation
                                                                   // here. If using a differential drivetrain, the
                                                                   // rotation will have no effect.
        );
        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return path;

    }

    public static PathPlannerPath createPath(Pose2d startPose, int aprilTagId, Rotation2d rotation) {
        return createPath(startPose, locationMap.get(aprilTagId),rotation );
    }

    public static Command followPath(Pose2d startPose, Pose2d endPose, Rotation2d rotation)
    {
        PathPlannerPath path = createPath(startPose, endPose, rotation);
        return AutoBuilder.followPath(path);
    }
}