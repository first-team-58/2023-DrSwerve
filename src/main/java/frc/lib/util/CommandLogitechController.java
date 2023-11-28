// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A version of {@link XboxController} with {@link Trigger} factories for command-based.
 *
 * @see XboxController
 */
@SuppressWarnings("MethodName")
public class CommandLogitechController extends CommandGenericHID {
  private final LogitechController m_hid;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandLogitechController(int port) {
    super(port);
    m_hid = new LogitechController(port);
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public LogitechController getHID() {
    return m_hid;
  }

  /**
   * Constructs an event instance around the joystick trigger's digital signal.
   *
   * @return an event instance representing the joystick trigger's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #joystickTrigger(EventLoop)
   */
  public Trigger joystickTrigger() {
    return joystickTrigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the joystick trigger's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the joystick trigger's digital signal attached to the given
   *     loop.
   */
  public Trigger joystickTrigger(EventLoop loop) {
    return m_hid.joystickTrigger(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around the joystick thumb's digital signal.
   *
   * @return an event instance representing the joystick thumb's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #joystickThumb(EventLoop)
   */
  public Trigger joystickThumb() {
    return joystickThumb(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the joystick thumb's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the joystick thumb's digital signal attached to the given
   *     loop.
   */
  public Trigger joystickThumb(EventLoop loop) {
    return m_hid.joystickThumb(loop).castTo(Trigger::new);
  }

  /**
   * Get the X axis value of joystick controller.
   *
   * @return The axis value.
   */
  public double getJoystickX() {
    return m_hid.getJoyStickX();
  }

  /**
   * Get the Y axis value of joystick controller.
   *
   * @return The axis value.
   */
  public double getJoystickY() {
    return m_hid.getJoyStickY();
  }

  /**
   * Get the Z axis value of joystick controller.
   *
   * @return The axis value.
   */
  public double getJoystickZ() {
    return m_hid.getJoyStickZ();
  }
}
