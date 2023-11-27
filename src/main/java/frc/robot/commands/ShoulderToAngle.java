package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.ShoulderConstraints;
import frc.robot.subsystems.Shoulder;

public class ShoulderToAngle extends CommandBase {
  private final Shoulder m_shoulder;
  private double m_goal;
  private double m_currentPosition;
  private boolean m_movingFrontToBack;

  /**
   * Creates a new SimpleShoulderToAngle.
   *
   * @param subsystem The shoulder used by this command.
   * @param angleFrontToBack The angle to move the shoulder from front to back
   */
  public ShoulderToAngle(Shoulder subsystem, double angle) {
    m_shoulder = subsystem;
    setup(subsystem);
    m_goal = angle;
    m_movingFrontToBack = true;
  }

  /**
   * Creates a new SimpleShoulderToAngle.
   *
   * @param subsystem The shoulder used by this command.
   * @param angleFrontToBack The angle to move the shoulder from front to back
   */
  public ShoulderToAngle(Shoulder subsystem, double angle, boolean movingBackToFront) {
    m_shoulder = subsystem;
    setup(subsystem);
    m_goal = angle;
    m_movingFrontToBack = false;
  }

  private void setup(Shoulder subsystem) {
    // configureShuffeboard();
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentPosition = m_shoulder.getArmAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_movingFrontToBack) m_shoulder.moveShoulderFrontToBack();
    else m_shoulder.moveShoulderBackToFront();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shoulder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int absError = Math.abs((int) (getCurrentError() - ShoulderConstraints.kShoulerAngleTolerance));
    return absError == 0;
  }

  private double getCurrentError() {
    return m_goal - m_currentPosition;
  }
}
