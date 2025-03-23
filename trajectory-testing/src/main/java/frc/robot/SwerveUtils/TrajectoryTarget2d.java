package frc.robot.SwerveUtils;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * This class helps to encapsulate information about a target trajectory's translation and rotation
 * information as well as its targeted waypoints.
 *
 * <p>In effect, this allows for the easy setting of target translation and rotation values and,
 * optionally, the addition of waypoints in the path of a robot's movement towards its targeted
 * end position. 
 */
public class TrajectoryTarget2d {
    private Translation2d translation;
    private Rotation2d rotation;
    private ArrayList<Translation2d> waypoints;

    /**
     * Get the targeted translation value of the trajectory target.
     *
     * @return The target trajectory's final translation target.
     */
    public Translation2d getTranslation() {
        return translation;
    }

    /**
     * Get the targeted rotational value of the trajectory target.
     *
     * @return The target trajectory's final rotational target.
     */
    public Rotation2d getRotation() {
        return rotation;
    }

    /**
     * Get every stored waypoint in TrajectoryTarget2d.
     *
     * @return An ArrayList of every waypoint stored in an instance of TrajectoryTarget2d.
     */
    public ArrayList<Translation2d> getWaypoints() {
        return waypoints;
    }

    /**
     * Adds waypoints to the the trajectory target.
     *
     * @param waypoints Waypoints to be added to the target trajectory.
     */
    public void addWaypoints(Translation2d... waypoints) {
        for (int i = 0; i < waypoints.length; i++) {
            this.waypoints.add(waypoints[i]);
        }
    }

    /**
     * Constructs a new TrajectoryTarget2d that contains information about a trajectory's end point
     * as well as its waypoints
     *
     * <p>Note: if no waypoints are passed, the robot will simply be assumed to me moving directly
     * towards its targeted end position in a straight line.
     *
     * @param x The targeted X trajectory in metres.
     * @param y The targeted Y trajectory in metres.
     * @param rotation The robot's targeted rotation amount in radians.
     * @param waypoints Waypoints to be added along the robot's path towards its targeted end position.
     */
    public TrajectoryTarget2d(double x, double y, double rotation, Translation2d... waypoints) {
        this.rotation = new Rotation2d(rotation);
        this.translation = new Translation2d(x, y);
        this.waypoints = new ArrayList<>(Arrays.asList(waypoints));
    }
}
