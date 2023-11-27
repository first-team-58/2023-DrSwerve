package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.Controllers;
import frc.robot.commands.TeleopGripper;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Reacher;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = 2;

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Shoulder m_shoulder = new Shoulder();
  private final Reacher m_reacher = new Reacher();
  private final Gripper m_gripper = new Gripper();

  private final AutoBuilder m_autoBuilder =
      new AutoBuilder(s_Swerve, m_shoulder, m_gripper, m_reacher);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureSubsystemCommands();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    Controllers.driverController.button(7).onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    Controllers.driverController.button(1).onTrue(new InstantCommand(() -> s_Swerve.toggleSlow()));

    /* Operator Controls */
    /* Move Shoulder front to back with Y button */
    Controllers.operatorController
        .y()
        .onTrue(new RunCommand(m_shoulder::moveShoulderFrontToBack, m_shoulder))
        .onFalse(new InstantCommand(m_shoulder::stop, m_shoulder));

    /* Move Shoulder back to front with X button */
    Controllers.operatorController
        .x()
        .onTrue(new RunCommand(m_shoulder::moveShoulderBackToFront, m_shoulder))
        .onFalse(new InstantCommand(m_shoulder::stop, m_shoulder));

    /* Extend Reacher out with B button */
    Controllers.operatorController
        .b()
        .onTrue(new RunCommand(m_reacher::reacherOut, m_reacher))
        .onFalse(new InstantCommand(m_reacher::stop, m_reacher));

    /* Retract Reacher in with A button */
    Controllers.operatorController
        .a()
        .onTrue(new RunCommand(m_reacher::reacherIn, m_reacher))
        .onFalse(new InstantCommand(m_reacher::stop, m_reacher));
  }

  private void configureSubsystemCommands() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -Controllers.driverController.getRawAxis(translationAxis),
            () -> -Controllers.driverController.getRawAxis(strafeAxis),
            () -> Controllers.driverController.getRawAxis(rotationAxis),
            () -> Controllers.driverController.button(8).getAsBoolean()));

    m_gripper.setDefaultCommand(
        new TeleopGripper(
            m_gripper,
            () -> Controllers.operatorController.getRightTriggerAxis(),
            () -> Controllers.operatorController.getLeftTriggerAxis()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoBuilder.getCurrentAutoCommand();
  }
}
