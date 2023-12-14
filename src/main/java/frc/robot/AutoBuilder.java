package frc.robot;

import com.choreo.lib.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ReacherOutToPosition;
import frc.robot.commands.ShoulderToAngle;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Reacher;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;
import java.util.Map;

public class AutoBuilder {
  private final Swerve m_swerve;
  private final Shoulder m_shoulder;
  private final Gripper m_gripper;
  private final Reacher m_reacher;

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public AutoBuilder(
      frc.robot.subsystems.Swerve s_Swerve, Shoulder shoulder, Gripper gripper, Reacher reacher) {
    this.m_swerve = s_Swerve;
    this.m_shoulder = shoulder;
    this.m_gripper = gripper;
    this.m_reacher = reacher;

    buildAutonomousChooser();
    configureShuffeboard();
  }

  private void buildAutonomousChooser() {
    m_chooser.setDefaultOption("Choreo Drive Straight", driveStraightChoreo());
    m_chooser.addOption("Choreo Drive Straight", driveStraightChoreo());
    m_chooser.addOption("Score Cube", cubeShootAndScoreHigh().andThen(bringArmHome()));
  }

  private Command driveStraightChoreo() {
    // do not include .traj extension when referencing the file name
    // this file should be in: "{deployDirectory}/choreo/"
    ChoreoTrajectory trajectory = Choreo.getTrajectory("straightTrajectory");

    // Create a swerve command for the robot to follow the trajectory
    return Choreo.choreoSwerveCommand(
        trajectory,
        m_swerve::getPose,
        new PIDController(5, 0, 0),
        new PIDController(5, 0, 0),
        new PIDController(-10, 0, 0),
        (ChassisSpeeds speeds) ->
            m_swerve.drive(
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond,
                false,
                false),
        false,  //TODO: research why true throws an error
        m_swerve);
  }

  /* private Command weekZeroChargeClimb() {
    return new RunCommand(() -> m_driveBase.arcadeDrive(.6, 0), m_driveBase)
        .withTimeout(3)
        .andThen(new WaitCommand(1))
        .andThen(
            new RunCommand(() -> m_driveBase.arcadeDrive(-.6, 0), m_driveBase).withTimeout(2.1));
  } */

  public Command bringArmHome() {
    return bringReacherIn()
        .alongWith(
            new WaitCommand(.5)
                .andThen(new RunCommand(m_shoulder::moveShoulderBackToFrontSlow, m_shoulder)))
        .until(m_shoulder::getFrontLimitSwitch)
        .withTimeout(2.1)
        .andThen(new InstantCommand(m_shoulder::stop, m_shoulder));
  }

  public Command bringReacherIn() {
    return new RunCommand(m_reacher::reacherIn, m_reacher)
        .until(m_reacher::getReacherLimitSwitch)
        .withTimeout(2.1)
        .andThen(new InstantCommand(m_reacher::stop, m_reacher));
  }

  public Command tryToDrive() {
    /* Get Values, Deadband*/
    double translationVal = .5;
    double strafeVal = 0;
    double rotationVal = 0;

    return new RunCommand(
            () ->
                m_swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                    rotationVal * Constants.Swerve.maxAngularVelocity,
                    false,
                    false),
            m_swerve)
        .withTimeout(3);
  }

  public Command cubeShootAndScoreHigh() {
    return resetAngleToZero()
        .andThen(spinGripperInAtStart())
        .andThen(
            new ShoulderToAngle(m_shoulder, 73.0)
                .withTimeout(.6)
                .alongWith(
                    new WaitCommand(.5)
                        .andThen(new ReacherOutToPosition(m_reacher, 1.5).withTimeout(1.5))))
        .andThen(shootGripperAndStop());
  }

  public Command cubeShootAndScoreMiddle() {
    return resetAngleToZero()
        .andThen(spinGripperInAtStart())
        .andThen(
            new ShoulderToAngle(m_shoulder, 63.0)
                .withTimeout(.4)
                .alongWith(
                    new WaitCommand(.5)
                        .andThen(new ReacherOutToPosition(m_reacher, 1).withTimeout(1))))
        .andThen(shootGripperAndStop());
  }

  private Command spinGripperInAtStart() {
    return new InstantCommand(() -> m_gripper.spin(.1), m_gripper);
  }

  private Command shootGripperAndStop() {
    return new RunCommand(() -> m_gripper.spin(-.9), m_gripper)
        .withTimeout(.5)
        .andThen(new InstantCommand(m_gripper::stop, m_gripper));
  }

  private Command resetAngleToZero() {
    return new InstantCommand(m_shoulder::resetAngleToZero, m_shoulder);
  }

  private void configureShuffeboard() {
    configureAutoBuilder();
  }

  private void configureAutoBuilder() {
    ShuffleboardTab tab = Shuffleboard.getTab("Auto Builder");

    /* add the auto chooser */
    tab.add("Auto Chooser", m_chooser).withPosition(6, 0).withSize(3, 1);

    configureShoulder();
  }

  private void configureShoulder() {

    /* Shoulder */
    ShuffleboardTab tab = Shuffleboard.getTab("Shoulder");
    tab.addBoolean("Front Switch", m_shoulder::getFrontLimitSwitch)
        .withPosition(0, 0)
        .withSize(1, 1);
    tab.addBoolean("Back Switch", m_shoulder::getBackLimitSwitch).withPosition(1, 0).withSize(1, 1);
    tab.addDouble("Shoulder Encoder", m_shoulder::getEncoderCount)
        .withPosition(0, 1)
        .withSize(1, 1);
    tab.addDouble("Shoulder Dial", m_shoulder::getArmAngle)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", -270, "max", 270))
        .withPosition(0, 2)
        .withSize(1, 1);
    tab.addDouble("Shoulder Angle", m_shoulder::getArmAngle).withPosition(1, 2).withSize(1, 1);
    tab.addDouble("Shoulder Speed", m_shoulder::getShoulderSpeed)
        .withPosition(2, 2)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kNumberSlider);
  }

  public Command getCurrentAutoCommand() {
    return m_chooser.getSelected();
  }
}
