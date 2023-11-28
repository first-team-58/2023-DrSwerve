// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Handle input from Xbox 360 or Xbox One controllers connected to the Driver Station.
 *
 * <p>This class handles Xbox input that comes from the Driver Station. Each time a value is
 * requested the most recent value is returned. There is a single class instance for each controller
 * and the mapping of ports to hardware buttons depends on the code in the Driver Station.
 */
public class LogitechController extends GenericHID {
  /** Represents a digital button on an XboxController. */
  public enum Button {
    kJoystickTrigger(1),
    kJoystickThumb(2);

    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and if not a Bumper button append `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Bumper")) {
        return name;
      }
      return name + "Button";
    }
  }

  /** Represents an axis on an LogitechController. */
  public enum Axis {
    kJoystickX(0),
    kJoystickY(1),
    kJoystickZ(2);

    public final int value;

    Axis(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the axis, matching the relevant methods. This is done by
     * stripping the leading `k`, and if a trigger axis append `Axis`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the axis.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public LogitechController(final int port) {
    super(port);

    HAL.report(tResourceType.kResourceType_Joystick, port + 1);
  }

  /**
   * Get the X axis value of the joystick.
   *
   * @return The axis value.
   */
  public double getJoyStickX() {
    return getRawAxis(Axis.kJoystickX.value);
  }

  /**
   * Get the Y axis value of the joystick.
   *
   * @return The axis value.
   */
  public double getJoyStickY() {
    return getRawAxis(Axis.kJoystickY.value);
  }

  /**
   * Get the Z axis value of the joystick.
   *
   * @return The axis value.
   */
  public double getJoyStickZ() {
    return getRawAxis(Axis.kJoystickZ.value);
  }

  /**
   * Read the value of the joystick trigger button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getJoystickTrigger() {
    return getRawButton(Button.kJoystickTrigger.value);
  }

  /**
   * Whether the joystick trigger button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getJoystickTriggerPressed() {
    return getRawButtonPressed(Button.kJoystickTrigger.value);
  }

  /**
   * Whether the joystick trigger button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getJoystickTriggerReleased() {
    return getRawButtonReleased(Button.kJoystickTrigger.value);
  }

  /**
   * Constructs an event instance around the joystick trigger button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the joystick trigger button's digital signal attached to
   *     the given loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent joystickTrigger(EventLoop loop) {
    return new BooleanEvent(loop, this::getJoystickTrigger);
  }

  /**
   * Read the value of the joystick thumb button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getJoystickThumb() {
    return getRawButton(Button.kJoystickThumb.value);
  }

  /**
   * Whether the joystick thumb button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getJoystickThumbPressed() {
    return getRawButtonPressed(Button.kJoystickThumb.value);
  }

  /**
   * Whether the joystick thumb was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getJoystickThumbReleased() {
    return getRawButtonReleased(Button.kJoystickThumb.value);
  }

  /**
   * Constructs an event instance around the joystick thumb button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Y button's digital signal attached to the given
   *     loop.
   */
  @SuppressWarnings("MethodName")
  public BooleanEvent joystickThumb(EventLoop loop) {
    return new BooleanEvent(loop, this::getJoystickThumb);
  }
}
