// package frc.FRC6657.autonomous.routines.RedAlliance;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;

// public class RedThree extends SequentialCommandGroup{
//     public RedThree {
//         DrivetrainSubsystem drivetrain,
        
//     }
//     addCommands(
//         new ParallelRaceGroup(
//           new WaitUntilCommand(intake::ballDetected),
//           drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_2, true)
//         ).beforeStarting(
//           new ParallelCommandGroup(
//             new InstantCommand(pistons::extend),
//             new InstantCommand(intake::start)
//           )
//         )
//         .andThen(
//           new ParallelCommandGroup(
//             new InstantCommand(pistons::retract),
//             new InstantCommand(intake::stop)
//           )
//         ),
//         drivetrain.new TrajectoryFollowerCommand(PATH_TO_SHOT_1, false)
//         .beforeStarting(
//           new InstantCommand(() -> flywheel.setRPMTarget(1000))
//         )
//         .andThen(
//           new SequentialCommandGroup(
//             new WaitUntilCommand(flywheel::atTarget),
//             new InstantCommand(accelerator::start),
//             new WaitCommand(0.5)
//           ).andThen(
//             new ParallelCommandGroup(
//               new InstantCommand(accelerator::stop),
//               new InstantCommand(flywheel::stop)
//             )
//           )
//         ),
//         drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_3, false)
//         .beforeStarting(
//           new ParallelCommandGroup(
//             new InstantCommand(pistons::extend),
//             new InstantCommand(intake::start)
//           )
//         )
//         .andThen(
//           new ParallelCommandGroup(
//             new InstantCommand(pistons::retract),
//             new InstantCommand(intake::stop)
//           )
//         ),
//         drivetrain.new TrajectoryFollowerCommand(PATH_TO_SHOT_2, false)
//         .beforeStarting(
//           new InstantCommand(() -> flywheel.setRPMTarget(1000))
//         )
//         .andThen(
//           new SequentialCommandGroup(
//             new WaitUntilCommand(flywheel::atTarget),
//             new InstantCommand(accelerator::start),
//             new WaitCommand(0.5)
//           ).andThen(
//             new ParallelCommandGroup(
//               new InstantCommand(accelerator::stop),
//               new InstantCommand(flywheel::stop)
//             )
//           )
//         ),
//         drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_4_5, false)
//         .beforeStarting(
//           new ParallelCommandGroup(
//             new InstantCommand(pistons::extend),
//             new InstantCommand(intake::start)
//           )
//         )
//         .andThen(
//           new ParallelCommandGroup(
//             new InstantCommand(pistons::retract),
//             new InstantCommand(intake::stop)
//           )
//         ),
//         drivetrain.new TrajectoryFollowerCommand(PATH_TO_SHOT_3, false)
//         .beforeStarting(
//           new InstantCommand(() -> flywheel.setRPMTarget(1000))
//         )
//         .andThen(
//           new SequentialCommandGroup(
//             new WaitUntilCommand(flywheel::atTarget),
//             new InstantCommand(accelerator::start),
//             new WaitCommand(0.5)
//           ).andThen(
//             new ParallelCommandGroup(
//               new InstantCommand(accelerator::stop),
//               new InstantCommand(flywheel::stop)
//             )
//           )
//         ),
//         drivetrain.new TrajectoryFollowerCommand(PATH_TO_EXIT, false)
//       );
//     }
  
//     private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(3,6,List.of(
//       new Pose2d(8.91, 6.363, Rotation2d.fromDegrees(91.158)),
//       new Pose2d(8.962, 7.26, Rotation2d.fromDegrees(91.158))
//     ),
//     false,
//     "Red Five TWO PATH_TO_BALL_2"
//     );
  
//     private Trajectory PATH_TO_SHOT_1 = Trajectories.generateTrajectory(1.5,4,List.of(
//       new Pose2d(8.962, 7.26, Rotation2d.fromDegrees(95)),
//       new Pose2d(8.858, 5.66, Rotation2d.fromDegrees(70))
//     ),
//     true,
//     "Red Five TWO PATH_TO_SHOT_1"
//     );
  
//     private Trajectory PATH_TO_BALL_3 = Trajectories.generateTrajectory(3,4,List.of(
//       new Pose2d(8.858, 6.4, Rotation2d.fromDegrees(0)),
//       new Pose2d(10.185, 6.4, Rotation2d.fromDegrees(0)),
//       new Pose2d(11.5, 6.4, Rotation2d.fromDegrees(0))
//     ),
//     false,
//     "Red Five TWO PATH_TO_BALL_3"
//     );
  
//     private Trajectory PATH_TO_SHOT_2 = Trajectories.generateTrajectory(2,4,List.of(
//       new Pose2d(11.5, 7, Rotation2d.fromDegrees(40)),
//       new Pose2d(9.8, 5.5, Rotation2d.fromDegrees(40))
//     ),
//     true,
//     "Red Five TWO PATH_TO_SHOT_2"
//     );
  
//     private Trajectory PATH_TO_BALL_4_5 = Trajectories.generateTrajectory(1.5,4,List.of(
//       new Pose2d(13, 5.5, Rotation2d.fromDegrees(41.68)),
//       new Pose2d(15, 6.5, Rotation2d.fromDegrees(41.68))
//     ),
//     false,
//     "Red Five TWO PATH_TO_BALL_4_5"
//     );
  
//     private Trajectory PATH_TO_SHOT_3 = Trajectories.generateTrajectory(2,4,List.of(
//       new Pose2d(13, 7.5, Rotation2d.fromDegrees(40)),
//       new Pose2d(9.7, 5.5, Rotation2d.fromDegrees(40))
//     ),
//     true,
//     "Red Five TWO PATH_TO_SHOT_3"
//     );
  
//     private Trajectory PATH_TO_EXIT = Trajectories.generateTrajectory(4,4,List.of(
//       new Pose2d(9.8, 5.5, Rotation2d.fromDegrees(60)),
//       new Pose2d(10.5, 6.75, Rotation2d.fromDegrees(40))
//     ),
//     false,
//     "Red Five TWO PATH_TO_EXIT"
//     );
  
//   }
  
// }
