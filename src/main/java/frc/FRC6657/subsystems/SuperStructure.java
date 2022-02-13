// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
//import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.PickupSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

public class SuperStructure extends SubsystemBase {
  
  public final DrivetrainSubsystem drivetrain;
  public final PickupSubsystem pickup;
  //public final ExtensionSubsystem intakeExtension;
  //public final FlywheelSubsystem flywheel;
  

  public SuperStructure(
    DrivetrainSubsystem drivetrain,
    PickupSubsystem pickup
    //ExtensionSubsystem intakeExtension,
    //FlywheelSubsystem flywheel
  ) {
    this.drivetrain = drivetrain;
    this.pickup = pickup;
    //this.intakeExtension = intakeExtension;
    //this.flywheel = flywheel;
  }

  public class RunIntakeCommand extends SequentialCommandGroup {
    public RunIntakeCommand(){
      addRequirements(pickup, SuperStructure.this);
      addCommands(
        //new InstantCommand(intakeExtension::toggleState),
        new InstantCommand(pickup::run)
      );
    }
  }

  public class StopIntakeCommand extends SequentialCommandGroup{
    public StopIntakeCommand(){
      addRequirements(pickup, SuperStructure.this);
      addCommands(
        //new InstantCommand(intakeExtension::toggleState),
        new InstantCommand(pickup::stop)
      );
    }
  }

  // public class ShootCommand extends SequentialCommandGroup{
  //   public ShootCommand(){
  //     addRequirements(SuperStructure.this);
  //   }

  //   @Override
  //   public void execute() {
  //       if(flywheel.atTarget()){
  //         accelerator.run();
  //       }
  //   }
  // }

  // public class TrackCommand extends SequentialCommandGroup {

  //   public TrackCommand(){
  //     addRequirements(SuperStructure.this);
  //     addCommands(
  //       drivetrain.new VisionAimCommand(vision.getYaw(), vision.getDistance(), vision.hasTarget())
  //     );
  //   }

  // }
}
