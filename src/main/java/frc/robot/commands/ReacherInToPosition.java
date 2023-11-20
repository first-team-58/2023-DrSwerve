package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Reacher;

public class ReacherInToPosition extends CommandBase {
  private final Reacher m_reacher;
  private double m_goal;
  private double m_currentPosition;

  /**
   * Creates a new ReacherInToPosition.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReacherInToPosition(Reacher subsystem, double timePositionSeconds) {
    m_reacher = subsystem;
    m_goal = System.currentTimeMillis() + (timePositionSeconds * 1000);

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentPosition = System.currentTimeMillis();
    m_reacher.reacherIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_reacher.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int absError = (int) getCurrentError();
    return absError == 0;
  }

  private double getCurrentError() {
    return m_goal - m_currentPosition;
  }
}
