package frc.FRC6657.autonomous;

import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

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

    public static Trajectory TEST_p1 = generateTrajectory(
        3,
        2,
        List.of(
            new Pose2d(1, 3, Rotation2d.fromDegrees(0)),
            new Pose2d(3, 5, Rotation2d.fromDegrees(90))
        ),
        false,
        "Test"
    );

    
    public static Trajectory TEST_p2= generateTrajectory(
        3,
        2,
        List.of(
            new Pose2d(3, 5, Rotation2d.fromDegrees(90)),
            new Pose2d(5, 3, Rotation2d.fromDegrees(180))
        ),
        true,
        "Test"
    );

    public static Trajectory TEST = TEST_p1.concatenate(TEST_p2);

    public static Trajectory Two_Ball_Far_1 = generateTrajectory(
        3,
        3,
        List.of(
            new Pose2d(7.627, 1.929, Rotation2d.fromDegrees(-90)),
            new Pose2d(7.62, 0.68, Rotation2d.fromDegrees(-90))
        ),
        false,
        "2BF1"
    );

    public static Trajectory Two_Ball_Far_2 = generateTrajectory(
        3,
        2,
        List.of(
            new Pose2d(7.62, 0.68, Rotation2d.fromDegrees(-90)),
            new Pose2d(6, 0.88, Rotation2d.fromDegrees(55.3))
        ),
        true,
        "2BF2"
    );

    public static Trajectory Two_Ball_Far = Two_Ball_Far_1.concatenate(Two_Ball_Far_2);

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
