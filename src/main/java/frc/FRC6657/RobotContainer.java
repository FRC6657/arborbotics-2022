// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657;

import java.util.Arrays;
import java.util.HashMap;

import javax.management.MBeanServerPermission;

import com.fasterxml.jackson.databind.deser.impl.NullsAsEmptyProvider;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.FRC6657.autonomous.routines.RedAlliance.RedMidTwo;
import frc.FRC6657.autonomous.routines.RedAlliance.RedSingleSteal;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueCoopFour;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueDoubleSteal;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueFive;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueHangarThree;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueHangarTwo;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueMidTwo;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueSingleSteal;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueWallThree;
import frc.FRC6657.autonomous.routines.BlueAllience.BlueWallTwo;
import frc.FRC6657.autonomous.routines.RedAlliance.RedCoopFour;
import frc.FRC6657.autonomous.routines.RedAlliance.RedDoubleSteal;
import frc.FRC6657.autonomous.routines.RedAlliance.RedFive;
import frc.FRC6657.autonomous.routines.RedAlliance.RedMidTwo;
import frc.FRC6657.autonomous.routines.RedAlliance.RedWallThree;
import frc.FRC6657.autonomous.routines.RedAlliance.RedWallTwo;
import frc.FRC6657.autonomous.routines.RedAlliance.RedHangarTwo;
import frc.FRC6657.autonomous.routines.RedAlliance.RedHangarThree;
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
import frc.FRC6657.subsystems.shooter.interpolation.InterpolatingTable;
import frc.FRC6657.subsystems.vision.VisionSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

@SuppressWarnings("unused")
public class RobotContainer implements Loggable {

  private CommandXboxController mXboxController = new CommandXboxController(0);
  private Joystick mJoystick = new Joystick(1);

  /** Dont trust this it needs overhauled */
  private DriverProfile mProfile = new DriverProfile(
      Constants.Drivetrain.kMaxAttainableSpeed, // Max Speed m/s
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
  public final LiftSubsystem lift;
  public final VisionSubsystem vision;


  public final Trigger flywheelReady, flywheelActive, ballDetected, intakeActive;

  SendableChooser<SequentialCommandGroup[]> mAutoChooser = new SendableChooser<>();

  public RobotContainer() {

    // Subsystem Assignments
    vision = new VisionSubsystem();
    accelerator = new AcceleratorSubsystem();
    blinkin = new BlinkinSubsystem();
    extension = new ExtensionSubsystem();
    flywheel = new FlywheelSubsystem();
    intake = new IntakeSubsystem();
    lift = new LiftSubsystem();
    drivetrain = new DrivetrainSubsystem(mProfile, vision.visionSupplier);
    hood = new HoodSubsystem(vision.visionSupplier, drivetrain::getEstDistanceToTarget);

    // Triggers
    flywheelReady = new Trigger(flywheel::atTarget);
    flywheelActive = new Trigger(flywheel::active);
    ballDetected = new Trigger(intake::ballDetected);
    intakeActive = new Trigger(intake::active);

    flywheelReady.or(flywheelActive).or(ballDetected).or(intakeActive).whileActiveContinuous(
        () -> blinkin.setIndicator(new BlinkinIndicator[] {
            new BlinkinIndicator("Idle", Constants.BlinkinPriorities.kIdle, Constants.BlinkinColors.kIdle),
            new BlinkinIndicator("FlywheelReady",
                (flywheelReady.get() ? 1 : 0) * Constants.BlinkinPriorities.kFlywheelReady,
                Constants.BlinkinColors.kReadyFlywheel),
            new BlinkinIndicator("FlywheelActive",
                (flywheelActive.get() ? 1 : 0) * Constants.BlinkinPriorities.kFlywheelActive,
                Constants.BlinkinColors.kNotReadyFlywheel),
            new BlinkinIndicator("BallDetected",
                (ballDetected.get() ? 1 : 0) * Constants.BlinkinPriorities.kBallDetected,
                Constants.BlinkinColors.kBallDetected),
            new BlinkinIndicator("IntakeActive",
                (intakeActive.get() ? 1 : 0) * Constants.BlinkinPriorities.kIntakeActive,
                Constants.BlinkinColors.kIntake)
        }));

    flywheelReady.or(flywheelActive).or(ballDetected).or(intakeActive).whenInactive(
        () -> blinkin.setIndicator(new BlinkinIndicator[] {
            new BlinkinIndicator("Idle", Constants.BlinkinPriorities.kIdle, Constants.BlinkinColors.kIdle)
    }));

    drivetrain.setDefaultCommand(new RunCommand(() -> {
      drivetrain.teleopCurvatureDrive(
          -ArborMath.signumPow(mXboxController.getLeftY(), 1.2),
          ArborMath.signumPow(mXboxController.getRightX(), 1.2),
          mXboxController.getRightTrigger(),
          mXboxController.getLeftTrigger());
    }, drivetrain));

    configureButtonBindings();
    configureAutoChooser();

    if (RobotBase.isSimulation()) {
      spoofVision();
    }
  }

  private void spoofVision() {
    NetworkTableInstance.getDefault().getTable("photonvision").getEntry("version").setValue("v2022.1.4");
  }

  private void configureAutoChooser() {
    mAutoChooser.setDefaultOption("Nothing", new SequentialCommandGroup[]{null,null});

    mAutoChooser.addOption("2 Ball Wall", new SequentialCommandGroup[]{
      new RedWallTwo(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier),
      new BlueWallTwo(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier)
    });

    mAutoChooser.addOption("2 Ball Hangar", 
    new SequentialCommandGroup[] {
      new RedHangarTwo(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier),
      new BlueHangarTwo(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier)
    }
  );  

    mAutoChooser.addOption("2 Ball Single Steal", 
      new SequentialCommandGroup[] {
        new RedSingleSteal(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier),
        new BlueSingleSteal(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier)
      }
    );

    mAutoChooser.addOption("2 Ball Double Steal",
    new SequentialCommandGroup[] {
      new RedDoubleSteal(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier),
      new BlueDoubleSteal(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier)
    }
  );

    mAutoChooser.addOption("2 Ball Mid",
      new SequentialCommandGroup[]{
        new RedMidTwo(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier),
        new BlueMidTwo(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier)
      }
    );
    
    mAutoChooser.addOption("3 Ball Wall",
      new SequentialCommandGroup[] {
        new RedWallThree(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier),
        new BlueWallThree(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier)
      }
    );

    mAutoChooser.addOption("3 Ball Hangar",
      new SequentialCommandGroup[]{
        new RedHangarThree(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier),
        new BlueHangarThree(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier)
      }
    );

    mAutoChooser.addOption("4 Ball Cooperative",
      new SequentialCommandGroup[]{
        new RedCoopFour(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier), 
        new BlueCoopFour(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier)
      }
    );

    mAutoChooser.addOption("5/4 Ball",
      new SequentialCommandGroup[]{
        new RedFive(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier),
        new BlueFive(drivetrain, intake, extension, flywheel, accelerator, hood, vision.visionSupplier)
      }
    );

    mAutoChooser.addOption("HomeHood", 
      new SequentialCommandGroup[] {
        new SequentialCommandGroup(hood.new Home()),
        new SequentialCommandGroup(hood.new Home())
      }
    );

    SmartDashboard.putData(mAutoChooser);
    }

  private void configureButtonBindings() {


    mXboxController.rightBumper().whenHeld(
      drivetrain.new VisionAimAssist()
    );

    mXboxController.a().whenHeld(
      new ParallelCommandGroup(
        new StartEndCommand(intake::start, intake::stop, intake),
        new InstantCommand(extension::extend, extension)
      )
    ).whenReleased(
      new InstantCommand(extension::retract).beforeStarting(new WaitCommand(.5))
    );

    mXboxController.b().whenHeld(
      new ParallelCommandGroup(
        new StartEndCommand(accelerator::start, accelerator::stop, accelerator),
        new StartEndCommand(() -> flywheel.set(0.2), flywheel::stop, flywheel),
        new StartEndCommand(extension::extend, extension::retract, extension)
      )
    );

    mXboxController.x().whenHeld(
      new ParallelCommandGroup(
        new StartEndCommand(() -> flywheel.set(-0.5), flywheel::stop, flywheel)
      )
    );
    
    mXboxController.y().whenHeld(
      new ParallelCommandGroup(
        new StartEndCommand(() -> flywheel.set(-0.5), flywheel::stop, flywheel),
        new StartEndCommand(accelerator::start, accelerator::stop, accelerator)
      )
    );
    

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