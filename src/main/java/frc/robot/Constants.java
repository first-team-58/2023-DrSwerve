package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.CommandXboxExtended;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static final class Controllers {
    public static final CommandXboxExtended driverController = new CommandXboxExtended(0);
    public static final CommandXboxExtended operatorController = new CommandXboxExtended(1);
    public static final double kRumbleValue = .3;
    public static final double stickDeadband = 0.1;

    public static final class SlewRateLimiters {
      // slow joystick units per second
      public static final double kMoveSlewRate = 3;
      public static final double kRotateSlewRate = 2;
      public static final double kShoulderSlewRate = 2;
      public static final double kReacherSlewRate = 4;
      public static final double kSpeedSwitchSlewRate = 1;
    }
  }

  public static final class Swerve {
    public static final int pigeonID = 1;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants
        chosenModule = // DONE: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

    /* Drivetrain Constants */
    public static final double trackWidth =
        Units.inchesToMeters(22.9375); // DONE: Need to confirm this initial measurement
    public static final double wheelBase =
        Units.inchesToMeters(23.125); // DONE: Need to confirm this initial measurement
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    /* Values provided by running SYSID for Dr Funk Tank Drive */
    // TODO: This must be tuned to specific robot
    public static final double driveKS = (0.21217 / 12);
    public static final double driveKV = (2.1978 / 12);
    public static final double driveKA = (0.37316 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity =
        10.0; // TODO: This must be tuned to specific robot
    
    public static final double slowSpeed = 1; // Meters per second


    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 1;
      public static final boolean driveMotorInvert = false;
      public static final boolean angleMotorInvert = false;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(251.81);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              driveMotorInvert,
              angleMotorInvert,
              angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 2;
      public static final boolean driveMotorInvert = false;
      public static final boolean angleMotorInvert = false;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(335.30);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              driveMotorInvert,
              angleMotorInvert,
              angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 3;
      public static final boolean driveMotorInvert = false;
      public static final boolean angleMotorInvert = false;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(332.40);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              driveMotorInvert,
              angleMotorInvert,
              angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 4;
      public static final boolean driveMotorInvert = false;
      public static final boolean angleMotorInvert = false;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(100.37);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              driveMotorInvert,
              angleMotorInvert,
              angleOffset);
    }
  }

  public static final
  class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
    // to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ArmConstants {

    public static final class ArmMotorPorts {
      public static final int kShoulderLeaderPort = 18;
      public static final int kShoulderFollowerPort = 19;
      public static final int kGripperPort = 17;
      public static final int kReacherPort = 16;
    }

    public static final class LimitSwitches {
      public static final int kAngleLimitSwitch = 6;
      public static final int kReacherLimitSwitch = 7;
      public static final int kFrontLimitSwitch = 9;
      public static final int kBackLimitSwitch = 8;
    }

    public static final class ReacherConstraints {
      public static final int kReacherEncoderMin = 1000;
      public static final int kReacherEncoderMax = 2450;
      public static final double kReacherSlowSpeed = .5;
      public static final double kReacherLimitSwitchDebounceTime = 1.2;
      public static final double kReacherAngleSwitchDebounceTime = .5;
    }

    public static final class ShoulderConstraints {
      public static final double kShoulderGearRatio = 1285;
      public static final double kShoulderFrontStartAngle = -50.0;
      public static final double kTurnToleranceDeg = 2.0;
      public static final double kShoulerAngleTolerance = 5;
      public static final double kShoulderFrontAngleSlowMode = 40;
      public static final double kShoulderBackAngleSlowMode = 180;
      public static final double kShoulderAutoSpeedModifier = 0.65;
      public static final double kShoulderLimitSwitchDebounceTime = 0.1;
    }

    public static final class ArmSpeeds {
      public static final double kMaxOutputShoulderFastSpeed = .40;
      public static final double kMaxOutputShoulderSlowSpeed = .15;
      public static final double kMaxReacherSpeed = 1;
      public static final double kMaxGripperSpeed = 1;
    }

    // TODO: remove unused constants
    // added teleop constraints
    public static final class TeleopConstraints {
      public static double kMaxOutputFastSpeed = 1;
      public static final double kMaxOutputSlowSpeed = .5;
      public static final double kMaxOutputSlowRotateSpeed = .3;
      public static final double kMaxOutputCurveSpeed = .5;
      public static final double kMoveDeadzone = .2;
      public static final double kRotateDeadzone = .2;
      public static final double kGripperDeadzone = .02;
      public static final int kDelayFastMode = 40; // miliseconds
      public static final double kRotate180Seconds = .67;
      public static final double kRotateSpeed = 0.7;
    }
  }
}
