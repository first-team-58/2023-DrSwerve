// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants.TeleopConstraints;
import frc.robot.subsystems.Gripper;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class TeleopGripper extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Gripper m_gripper;

  private final DoubleSupplier m_spinInSupplier, m_spinOutSupplier;
  private boolean m_isFinished = false;

  /**
   * Creates a new TeleopDrive.
   *
   * @param gripper The subsystem used by this command.
   */
  public TeleopGripper(Gripper gripper, DoubleSupplier spinIn, DoubleSupplier spinOut) {
    m_gripper = gripper;
    m_spinInSupplier = spinIn;
    m_spinOutSupplier = spinOut;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0;

    if (m_spinInSupplier.getAsDouble() > 0) speed = -m_spinInSupplier.getAsDouble();
    else if (m_spinOutSupplier.getAsDouble() > 0) speed = m_spinOutSupplier.getAsDouble();

    double spinValue = calculateDeadzone(speed, TeleopConstraints.kGripperDeadzone);

    m_gripper.spin(spinValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }

  /**
   * Checks the joystick value against the deadzone, and returns 0 if the value is lower than the
   * deadzone
   *
   * @param value joystick value
   * @param deadzone deadzone value
   * @return 0 if Math.abs(value) < deadzone, value otherwise
   */
  private double calculateDeadzone(double value, double deadzone) {
    if (Math.abs(value) < deadzone) {
      return 0;
    }
    return value;
  }
}
