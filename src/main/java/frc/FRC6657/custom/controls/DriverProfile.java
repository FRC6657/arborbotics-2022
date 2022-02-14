package frc.FRC6657.custom.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.FRC6657.Constants;
import frc.FRC6657.custom.ctre.IdleMode;

public class DriverProfile {
    public double kMaxSpeed;
    public double kMaxTurn;
    public double kMaxTurnDegrees;
    public IdleMode kIdleMode;
    public int kSpeedModBtn;
    public double kModSpeed;
    public double kModTurn;
    public int kQuickturnBtn;

    public GenericHID mController;

    public int kDriveAxis;
    public int kTurnAxis;

    public ControlStyle kStyle;

    public DriverProfile(
        Joystick joystick,
        int driveAxis,
        int turnAxis,
        ControlStyle style,
        double kMaxSpeed,
        double kMaxTurn,
        IdleMode kIdleMode,
        int kSpeedModBtn,
        double kModSpeed,
        double kModTurn
    ){
        this.mController = joystick;
        this.kDriveAxis = driveAxis;
        this.kTurnAxis = turnAxis;
        this.kStyle = style;
        this.kMaxSpeed = kMaxSpeed;
        this.kMaxTurn = (Math.PI*Constants.Drivetrain.kTrackWidth)/(360/kMaxTurn);
        this.kMaxTurnDegrees = kMaxTurn;
        this.kIdleMode = kIdleMode;
        this.kSpeedModBtn = kSpeedModBtn;
        this.kModSpeed = kModSpeed;
        this.kModTurn = kModTurn;
    }

    public DriverProfile(
        XboxController controller,
        int driveAxis,
        int turnAxis,
        ControlStyle style,
        double kMaxSpeed,
        double kMaxTurn,
        IdleMode kIdleMode,
        int kSpeedModBtn,
        double kModSpeed,
        double kModTurn,
        int kQuickturnBtn
    ){
        this.mController = controller;
        this.kDriveAxis = driveAxis;
        this.kTurnAxis = turnAxis;
        this.kMaxTurnDegrees = kMaxTurn;
        this.kStyle = style;
        this.kMaxSpeed = kMaxSpeed;
        this.kMaxTurn = (Math.PI*Constants.Drivetrain.kTrackWidth)/(360/kMaxTurn);
        this.kIdleMode = kIdleMode;
        this.kSpeedModBtn = kSpeedModBtn;
        this.kModSpeed = kModSpeed;
        this.kModTurn = (Math.PI*Constants.Drivetrain.kTrackWidth)/(360/kModTurn);
        this.kQuickturnBtn = kQuickturnBtn;
    }

}
