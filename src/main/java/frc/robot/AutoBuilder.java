package frc.robot;

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

public class AutoBuilder {
  private final Shoulder m_shoulder;
  private final Gripper m_gripper;
  private final Reacher m_reacher;

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public AutoBuilder(Shoulder shoulder, Gripper gripper, Reacher reacher) {
    this.m_shoulder = shoulder;
    this.m_gripper = gripper;
    this.m_reacher = reacher;

    buildAutonomousChooser();
  }

  private void buildAutonomousChooser() {
    m_chooser.setDefaultOption("DO NOTHING", new PrintCommand("NOTHING"));
    m_chooser.addOption("Score Cube", cubeShootAndScoreHigh());
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
