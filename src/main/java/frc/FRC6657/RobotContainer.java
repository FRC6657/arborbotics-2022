// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657;

import java.util.Arrays;
import java.util.HashMap;

import javax.management.MBeanServerPermission;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.FRC6657.autonomous.routines.BallDetectionTest;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueFive;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueMidTwo;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueTopTwo;
import frc.FRC6657.autonomous.routines.RedAlliance.RedFive;
import frc.FRC6657.autonomous.routines.RedAlliance.RedMidTwo;
import frc.FRC6657.autonomous.routines.RedAlliance.RedTopTwo;
import frc.FRC6657.custom.ArborMath;
import frc.FRC6657.custom.controls.CommandXboxController;
import frc.FRC6657.custom.controls.Deadbander;
import frc.FRC6657.custom.controls.DriverProfile;
import frc.FRC6657.custom.rev.Blinkin;
import frc.FRC6657.custom.rev.BlinkinIndicator;
import frc.FRC6657.subsystems.blinkin.BlinkinSubsystem;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.lift.LiftSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

@SuppressWarnings("unused")
public class RobotContainer implements Loggable{

  private CommandXboxController mXboxController = new CommandXboxController(0);
  private Joystick mJoystick = new Joystick(1);

  /** Dont trust this it needs overhauled */
  private DriverProfile mProfile = new DriverProfile(
    5d, // Max Speed m/s
    90d, // Max Turn deg/s
    3d, // Mod Drive Speed m/s
    80d // Mod Turn Speed deg/s
  );

  private String Controls = "Testing";

  public final AcceleratorSubsystem accelerator;
  public final BlinkinSubsystem blinkin;
  public final DrivetrainSubsystem drivetrain;
  public final ExtensionSubsystem extension;
  public final FlywheelSubsystem flywheel;
  public final HoodSubsystem hood;
  public final IntakeSubsystem intake;
  //public final LiftSubsystem lift;
  //public final VisionSubsystem vision;

  public final Trigger flywheelReady,flywheelActive,ballDetected,intakeActive;

  SendableChooser<SequentialCommandGroup[]> mAutoChooser = new SendableChooser<>();

  public RobotContainer() {

    //Subsystem Assignments
    accelerator = new AcceleratorSubsystem();
    blinkin = new BlinkinSubsystem();
    drivetrain = new DrivetrainSubsystem(mProfile);
    extension = new ExtensionSubsystem();
    flywheel = new FlywheelSubsystem();
    hood = new HoodSubsystem();
    intake = new IntakeSubsystem();
    //lift = new LiftSubsystem();
    //vision = new VisionSubsystem();

    //Triggers
    flywheelReady = new Trigger(flywheel::atTarget);
    flywheelActive = new Trigger(flywheel::active);
    ballDetected = new Trigger(intake::ballDetected);
    intakeActive = new Trigger(intake::active);

    flywheelReady.or(flywheelActive).or(ballDetected).or(intakeActive).whileActiveContinuous(
      () -> blinkin.setIndicator(new BlinkinIndicator[] {
        new BlinkinIndicator("Idle", Constants.BlinkinPriorities.kIdle, Constants.BlinkinColors.kIdle),
        new BlinkinIndicator("FlywheelReady", (flywheelReady.get() ? 1 : 0) * Constants.BlinkinPriorities.kFlywheelReady, Constants.BlinkinColors.kReadyFlywheel),
        new BlinkinIndicator("FlywheelActive", (flywheelActive.get() ? 1 : 0) * Constants.BlinkinPriorities.kFlywheelActive, Constants.BlinkinColors.kNotReadyFlywheel),
        new BlinkinIndicator("BallDetected", (ballDetected.get() ? 1 : 0) * Constants.BlinkinPriorities.kBallDetected, Constants.BlinkinColors.kBallDetected),
        new BlinkinIndicator("IntakeActive", (intakeActive.get() ? 1 : 0) * Constants.BlinkinPriorities.kIntakeActive, Constants.BlinkinColors.kIntake)
      })
    );

    flywheelReady.or(flywheelActive).or(ballDetected).or(intakeActive).whenInactive(
      () -> blinkin.setIndicator(new BlinkinIndicator[] {
        new BlinkinIndicator("Idle", Constants.BlinkinPriorities.kIdle, Constants.BlinkinColors.kIdle)
      })
    );


    drivetrain.setDefaultCommand(new RunCommand(() -> {
      drivetrain.teleopCurvatureDrive(
          -ArborMath.signumPow(mXboxController.getLeftY(), 1.2),
          ArborMath.signumPow(mXboxController.getRightX(), 1.2),
          mXboxController.getRightTrigger(),
          mXboxController.getLeftTrigger());
    }, drivetrain));
    
    configureButtonBindings();
    configureAutoChooser();
  }

  private void configureAutoChooser() {
    mAutoChooser.setDefaultOption("Nothing", new SequentialCommandGroup[]{null,null});
    mAutoChooser.addOption("Middle 2",
      new SequentialCommandGroup[]{
        new RedMidTwo(drivetrain, intake, extension, flywheel, accelerator),
        new BlueMidTwo(drivetrain, intake, extension, flywheel, accelerator)
      }
    );

    mAutoChooser.addOption("5",
      new SequentialCommandGroup[]{
        new RedFive(drivetrain, intake, extension, flywheel, accelerator),
        new BlueFive(drivetrain, intake, extension, flywheel, accelerator)
      }
    );

    mAutoChooser.addOption("Top 2", 
      new SequentialCommandGroup[] {
        new RedTopTwo(drivetrain, intake, extension, flywheel, accelerator),
        new BlueTopTwo(drivetrain, intake, extension, flywheel, accelerator)
      }
      );  


    SmartDashboard.putData(mAutoChooser);}

  private void configureButtonBindings() {
    
    switch(Controls){
      case "Testing":
        //Intake
        mXboxController.a().whenHeld(
          new ParallelCommandGroup(
            new StartEndCommand(
              () -> intake.set(Constants.Intake.kSpeed), 
              intake::stop,
              intake
            ),
            new StartEndCommand(
              extension::extend,
              extension::retract,
              extension
            )
          )
        );

      //Flywheel
      mXboxController.b().whenHeld(
          new StartEndCommand(
            () -> flywheel.setRPMTarget(1000),
            flywheel::stop,
            flywheel
          )
      );

      //Hood
      mXboxController.pov.up().whenHeld(
        new StartEndCommand(
          () -> hood.set(Constants.Hood.kUpSpeed), 
          hood::stop,
          hood
        )
      );

      mXboxController.pov.down().whenHeld(
        new StartEndCommand(
          () -> hood.set(Constants.Hood.kDownSpeed), 
          hood::stop,
          hood
        )
      );

    }
  }

  public SequentialCommandGroup getAutonomousCommand() {
    int alliance = 0;
    if(DriverStation.getAlliance() == Alliance.Red){
      alliance = 0;
    }else{
      alliance = 1;
    }
    return mAutoChooser.getSelected()[alliance];
  }
}
