package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.NEBalance;
import frc.robot.commands.ReacherOutToPosition;
import frc.robot.commands.ShoulderToAngle;
import frc.robot.commands.autonomous.Balance;
import frc.robot.commands.autonomous.DriveDistance;
import frc.robot.commands.autonomous.NewBalance;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Reacher;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;

public class AutoBuilder {
  private final Swerve m_driveBase;
  private final Shoulder m_shoulder;
  private final Gripper m_gripper;
  private final Reacher m_reacher;
  private final Leds m_led;

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static GenericEntry maxSpeedEntry,
      kpCTEntry,
      MinPowerCTEntry,
      MoveModifierCTEntry,
      CollectedAreaCTEntry,
      CollectedYCTEntry,
      CollectedXCTEntry,
      FinalTagDistanceEntry,
      TagTurnTolerenceEntry;

  public AutoBuilder(
      Swerve driveBase, Shoulder shoulder, Gripper gripper, Reacher reacher, Leds led) {
    this.m_driveBase = driveBase;
    this.m_shoulder = shoulder;
    this.m_gripper = gripper;
    this.m_reacher = reacher;
    this.m_led = led;

    buildAutonomousChooser();
  }

  private void buildAutonomousChooser() {}

  /* private Command weekZeroChargeClimb() {
    return new RunCommand(() -> m_driveBase.arcadeDrive(.6, 0), m_driveBase)
        .withTimeout(3)
        .andThen(new WaitCommand(1))
        .andThen(
            new RunCommand(() -> m_driveBase.arcadeDrive(-.6, 0), m_driveBase).withTimeout(2.1));
  } */

  private Command scoreCubeAndBalanceUNH() {
    return cubeShootAndScoreHigh()
        .andThen(
            driveBackwardWithDistanceAndSpeed(3.5, .5)
                .alongWith(bringArmHome())
                .andThen(new WaitCommand(.5))
                .andThen(balanceOnChargeStationWeek4()));
  }

  private Command scoreCubeAndDriveBackOneMeter() {
    return cubeShootAndScoreHigh().andThen(driveBackwardForDistance(1.0).alongWith(bringArmHome()));
  }

  private Command scoreCubeAndDriveBackThreeMeters() {
    return cubeShootAndScoreHigh().andThen(driveBackwardForDistance(3.5).alongWith(bringArmHome()));
  }

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

  public Command getCurrentAutoCommand() {
    return m_chooser.getSelected();
  }
}
