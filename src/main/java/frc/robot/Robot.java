package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot {

  private Command mAutonomousCommand;
  private RobotContainer mRobotContainer;

  @Override
  public void robotInit() {
    mRobotContainer = new RobotContainer();
    Logger.configureLoggingAndConfig(mRobotContainer, false);
  }

  @Override
  public void robotPeriodic() {
    //NetworkTableInstance.getDefault().flush();
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
    mRobotContainer.updateField();
  }

  @Override
  public void disabledInit() {
    mRobotContainer.stopAll().schedule();
  }

  @Override
  public void autonomousInit() {
    mAutonomousCommand = mRobotContainer.getAutonomousCommand();
    if (mAutonomousCommand != null) {
      mAutonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (mAutonomousCommand != null) {
      mAutonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationInit() {
      DriverStation.silenceJoystickConnectionWarning(true);
  }

}
