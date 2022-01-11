package frc.robot.autonomous;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

public class Trajectories {
    
    public static Trajectory GENERATE = generateTrajectory(
        1,
        1,
        List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(0, 0.1, Rotation2d.fromDegrees(0))
        ),
        true,
        "GENERATE"
    );

    public static Trajectory TEST = generateTrajectory(
        2,
        1,
        List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(Units.feetToMeters(8), Units.feetToMeters(3), Rotation2d.fromDegrees(90)),
            new Pose2d(Units.feetToMeters(8), Units.feetToMeters(4), Rotation2d.fromDegrees(90))
        ),
        false,
        "Test"
    );

    public static Trajectory planner = PathPlanner.loadPath("Test Path", 4, 2);

    /**
     * 
     * Trajectory Generator Function
     * 
     * @param maxVel Max Velocity m/s
     * @param maxAccel Max Accel m/s²
     * @param waypoints Points to create a path to
     * @param reversed Whether you drive forward or backward
     * @param name Label for print purposes
     * @return A Trajectory
     */
    private static Trajectory generateTrajectory(double maxVel, double maxAccel, List<Pose2d> waypoints, boolean reversed, String name) {
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        config.setReversed(reversed);
        System.out.println("Trajectory '" + name + "' Generated");
        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }
}
