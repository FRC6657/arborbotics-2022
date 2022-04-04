package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.common.FireOne;
import frc.robot.autonomous.common.FireTwo;
import frc.robot.autonomous.routines.BlueAllience.BlueFenderFive;
import frc.robot.autonomous.routines.BlueAllience.BlueFenderThree;
import frc.robot.autonomous.routines.BlueAllience.BlueFenderTwoHanger;
import frc.robot.autonomous.routines.BlueAllience.BlueFenderTwoMid;
import frc.robot.autonomous.routines.BlueAllience.BlueFenderTwoWall;
import frc.robot.autonomous.routines.BlueAllience.Taxi;
import frc.robot.autonomous.routines.RedAlliance.RedFenderFive;
import frc.robot.autonomous.routines.RedAlliance.RedFenderThree;
import frc.robot.autonomous.routines.RedAlliance.RedFenderTwoHanger;
import frc.robot.autonomous.routines.RedAlliance.RedFenderTwoMid;
import frc.robot.autonomous.routines.RedAlliance.RedFenderTwoWall;
import frc.robot.autonomous.routines.RedAlliance.RedTwoHanger;
import frc.robot.autonomous.routines.test.RoutineTesting;
import frc.robot.custom.ArborMath;
import frc.robot.custom.controls.CommandXboxController;
import frc.robot.custom.controls.Deadbander;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.Interpolation.InterpolatingTable;

public class RobotContainer {

  private VisionSubsystem vision = new VisionSubsystem();
  private AcceleratorSubsystem accelerator = new AcceleratorSubsystem();
  private LiftSubsystem lift = new LiftSubsystem();
  private IntakeSubsystem intake = new IntakeSubsystem();
  private IntakePistonsSubsystem pistons = new IntakePistonsSubsystem();
  private FlywheelSubsystem flywheel = new FlywheelSubsystem(vision.visionSupplier);
  private HoodSubsystem hood = new HoodSubsystem(vision.visionSupplier);
  private DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(vision.visionSupplier);

  private Trigger intakeExtended = new Trigger(pistons::extended);

  private SendableChooser<SequentialCommandGroup[]> mAutoChooser = new SendableChooser<>();

  private static Field2d mField = new Field2d();
  private FieldObject2d mIntakeVisualizer = mField.getObject("Intake");
  private SlewRateLimiter mIntakeAnimator = new SlewRateLimiter(1);

  private CommandXboxController mDriverController = new CommandXboxController(0);
  private CommandXboxController mOperatorController = new CommandXboxController(1);
  
  public RobotContainer() {

    LiveWindow.disableAllTelemetry();

    configureButtonBindings();
    configureAutoChooser();

    SmartDashboard.putData(mField);

    drivetrain.setDefaultCommand(
      drivetrain.new TeleOpCommand(
        () -> ArborMath.signumPow(Deadbander.applyLinearScaledDeadband(-mDriverController.getLeftY(), 0.075), 1.2),
        () -> ArborMath.signumPow(Deadbander.applyLinearScaledDeadband(-mDriverController.getRightX(), 0.075), 1.2), 
        () -> mDriverController.getLeftTrigger()
      )
    );

    NetworkTableInstance.getDefault().getTable("photonvision").getEntry("version").setValue("v2022.1.6");

  }

  private void configureButtonBindings() {

    mOperatorController.a().whenHeld(
      new ParallelCommandGroup(
        new StartEndCommand(
          pistons::extend, 
          pistons::retract,
          pistons
        ),
        new StartEndCommand(
          intake::start,
          intake::stop,
          intake
        )
      )
    );

    mOperatorController.x().whenHeld(
      new StartEndCommand(accelerator::start, accelerator::stop, accelerator)
    );

    mOperatorController.pov.up().whenHeld(
      new ParallelCommandGroup(
        new InstantCommand(hood::stop, hood),
        new StartEndCommand(
          () -> lift.set(1), 
          lift::stop, lift
        )
      )
    );

    mOperatorController.pov.down().whenHeld(
      new ParallelCommandGroup(
        new InstantCommand(hood::stop, hood),
        new StartEndCommand(
          () -> lift.set(-1), 
          lift::stop, 
          lift
        )
      )
    );


    mDriverController.rightBumper().whenHeld(
      new StartEndCommand(
        () -> hood.setTargetAngle(InterpolatingTable.get(vision.visionSupplier.getPitch()).hoodAngle), hood::stop, hood
      ).beforeStarting(
        new PrintCommand(Double.toString(vision.visionSupplier.getPitch()))
      )
    );

    //TODO Test
    mDriverController.leftBumper().whenHeld(
      drivetrain.new VisionAimAssist()
        .beforeStarting(
          new SequentialCommandGroup(
            new InstantCommand(() -> vision.visionSupplier.enableLEDs()),
            new WaitCommand(0.25)
          )
          ).andThen(new InstantCommand(() -> vision.visionSupplier.disableLEDs()))
    );
    


    //Fender Shot
    mDriverController.x().whenHeld(
      new SequentialCommandGroup(
          new ParallelCommandGroup(
            new InstantCommand(() -> flywheel.setTargetRPM(3150), flywheel),
            new InstantCommand(() -> hood.setTargetAngle(3), hood)
          ),
          new WaitUntilCommand(() -> flywheel.ready()),
          new InstantCommand(pistons::extend, pistons),
          new RunCommand(accelerator::start, accelerator).withInterrupt(() -> flywheel.shotDetector()),
          new ParallelCommandGroup(
            new InstantCommand(accelerator::reverse),
            new InstantCommand(pistons::retract)
          ),
          new WaitCommand(0.25),
          new InstantCommand(accelerator::stop),
          new WaitUntilCommand(() -> flywheel.ready()),
          new RunCommand(accelerator::start)
      )
    ).whenReleased(
      new ParallelCommandGroup(
        new InstantCommand(flywheel::stop, flywheel),
        new InstantCommand(hood::stop, hood),
        new InstantCommand(accelerator::stop, accelerator),
        new InstantCommand(pistons::retract, pistons)
      )
    );


    mOperatorController.y().whenHeld(
      new FireOne(flywheel, hood, accelerator, pistons, 1000, 1)
    ).whenReleased(
      new ParallelCommandGroup(
          new InstantCommand(flywheel::stop, flywheel),
          new InstantCommand(hood::stop, hood),
          new InstantCommand(accelerator::stop, accelerator),
          new InstantCommand(pistons::retract, pistons)
      )
  );


    mOperatorController.b().whenHeld(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> flywheel.setTargetRPM(1250), flywheel),
                new InstantCommand(() -> hood.setTargetAngle(15), hood)),
            new WaitUntilCommand(() -> flywheel.ready()),
            new InstantCommand(pistons::extend, pistons),
            new RunCommand(accelerator::start, accelerator).withInterrupt(() -> flywheel.shotDetector()),
            new InstantCommand(() -> accelerator.set(-1)),
            new WaitCommand(0.25),
            new InstantCommand(accelerator::stop),
            new WaitUntilCommand(() -> flywheel.ready()),
            new RunCommand(accelerator::start)))
        .whenReleased(
            new ParallelCommandGroup(
                new InstantCommand(flywheel::stop, flywheel),
                new InstantCommand(hood::stop, hood),
                new InstantCommand(accelerator::stop, accelerator),
                new InstantCommand(pistons::retract, pistons)
            )
        );

    // Tarmac Shot
    mDriverController.b().whenHeld(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> flywheel.setTargetRPM(InterpolatingTable.get(vision.visionSupplier.getPitch()).rpm), flywheel),
                new InstantCommand(() -> hood.setTargetAngle(InterpolatingTable.get(vision.visionSupplier.getPitch()).hoodAngle), hood)),
            new WaitUntilCommand(() -> flywheel.ready()),
            new InstantCommand(pistons::extend, pistons),
            new InstantCommand(intake::start),
            new RunCommand(accelerator::start, accelerator).withInterrupt(() -> flywheel.shotDetector()),
            new InstantCommand(() -> accelerator.set(-1)),
            new WaitCommand(0.25),
            new InstantCommand(accelerator::stop),
            new WaitUntilCommand(() -> flywheel.ready()),
            new RunCommand(accelerator::start)))
        .whenReleased(
            new ParallelCommandGroup(
                new InstantCommand(flywheel::stop, flywheel),
                new InstantCommand(hood::stop, hood),
                new InstantCommand(accelerator::stop, accelerator),
                new InstantCommand(pistons::retract, pistons),
                new InstantCommand(intake::stop, intake)
            )
        );

  }
  
  public void configureAutoChooser(){
    
    mAutoChooser.setDefaultOption("Nothing", new SequentialCommandGroup[]{null,null});

    mAutoChooser.addOption("FenderFive", new SequentialCommandGroup[]{
      new RedFenderFive(drivetrain, intake, pistons, accelerator, flywheel, hood),
      new BlueFenderFive(drivetrain, intake, pistons, accelerator, flywheel, hood)
    });

    mAutoChooser.addOption("FenderThree", new SequentialCommandGroup[]{
      new RedFenderThree(drivetrain, intake, pistons, flywheel, hood, accelerator),
      new BlueFenderThree(drivetrain, intake, pistons, accelerator, flywheel, hood)
    });

    mAutoChooser.addOption("FenderTwoHanger", new SequentialCommandGroup[]{
      new RedFenderTwoHanger(drivetrain, intake, pistons, flywheel, hood, accelerator),
      new BlueFenderTwoHanger(drivetrain, intake, pistons, flywheel, hood, accelerator)
    });

    mAutoChooser.addOption("FenderTwoMid", new SequentialCommandGroup[]{
      new RedFenderTwoMid(drivetrain, intake, pistons, flywheel, hood, accelerator),
      new BlueFenderTwoMid(drivetrain, intake, pistons, flywheel, hood, accelerator)
    });
    
    mAutoChooser.addOption("FenderTwoWall", new SequentialCommandGroup[]{
      new RedFenderTwoWall(drivetrain, intake, pistons, flywheel, hood, accelerator),
      new BlueFenderTwoWall(drivetrain, intake, pistons, flywheel, hood, accelerator)
    });

    mAutoChooser.addOption("Taxi", new SequentialCommandGroup[]{
      new Taxi(drivetrain, intake, pistons, accelerator, flywheel, hood, vision.visionSupplier),
      new Taxi(drivetrain, intake, pistons, accelerator, flywheel, hood, vision.visionSupplier)
    });

    mAutoChooser.addOption("2 Ball Hangar", new SequentialCommandGroup[]{
      new RedTwoHanger(drivetrain, intake, pistons, flywheel, hood, accelerator, vision.visionSupplier),
      new RedTwoHanger(drivetrain, intake, pistons, flywheel, hood, accelerator, vision.visionSupplier)
    });

    SmartDashboard.putData("Auto Chooser", mAutoChooser);

  }

  public void updateField(){
    if(intakeExtended.getAsBoolean()){
      mIntakeVisualizer.setPose(
        new Pose2d(
          (drivetrain.getRobotPosition().getX() + mIntakeAnimator.calculate(0.6056505) * Math.cos(Units.degreesToRadians(drivetrain.getRobotPosition().getRotation().getDegrees()))),
          (drivetrain.getRobotPosition().getY() + mIntakeAnimator.calculate(0.6056505) * Math.sin(Units.degreesToRadians(drivetrain.getRobotPosition().getRotation().getDegrees()))),
          drivetrain.getRobotPosition().getRotation()
        )
      );
    }else{
      mIntakeVisualizer.setPose(
        new Pose2d(
          (drivetrain.getRobotPosition().getX() + mIntakeAnimator.calculate(0.3796505) * Math.cos(Units.degreesToRadians(drivetrain.getRobotPosition().getRotation().getDegrees()))),
          (drivetrain.getRobotPosition().getY() + mIntakeAnimator.calculate(0.3796505) * Math.sin(Units.degreesToRadians(drivetrain.getRobotPosition().getRotation().getDegrees()))),
          drivetrain.getRobotPosition().getRotation()
        )
      );
    }

    mField.setRobotPose(drivetrain.getRobotPosition());

    vision.visionSupplier.processSim(drivetrain.getRobotPosition());

  }


  public SequentialCommandGroup getAutonomousCommand() {

    int alliance = 0;
    if(DriverStation.getAlliance() == Alliance.Red){
      alliance = 0;
    }else{
      alliance = 1;
    }
    //return new RoutineTesting(drivetrain, intake, pistons, flywheel, hood, accelerator, vision.visionSupplier);
    return mAutoChooser.getSelected()[alliance];
    //return new BlueFenderTwoHanger(drivetrain, intake, pistons, flywheel, hood, accelerator);
  }

  public static Field2d getField(){ 
    return mField;
  }

  public Command stopAll(){
      return new ParallelCommandGroup(
        new InstantCommand(intake::stop, intake),
        new InstantCommand(pistons::retract, pistons),
        new InstantCommand(accelerator::stop, accelerator),
        new InstantCommand(lift::stop, lift),
        new InstantCommand(flywheel::stop, flywheel),
        new InstantCommand(vision.visionSupplier::disableLEDs, vision),
        new InstantCommand(drivetrain::stop, drivetrain),
        new InstantCommand(hood::stop, hood),
        new PrintCommand("Robot Disabled")
      );
  }
}
