package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.ArmMotorPorts;
import frc.robot.Constants.ArmConstants.ArmSpeeds;
import frc.robot.Constants.ArmConstants.LimitSwitches;
import frc.robot.Constants.ArmConstants.ReacherConstraints;

public class Reacher extends SubsystemBase {
  /** Creates a new Reacher Subsystem. */
  private final TalonSRX m_extender;

  private boolean m_overrideLimit = false;

  DigitalInput m_reacherLimitSwitch = new DigitalInput(LimitSwitches.kReacherLimitSwitch);
  DigitalInput m_angleLimitSwitch = new DigitalInput(LimitSwitches.kAngleLimitSwitch);

  Debouncer m_reacherDebouncer =
      new Debouncer(ReacherConstraints.kReacherLimitSwitchDebounceTime, DebounceType.kFalling);

  Debouncer m_angleDebouncer =
      new Debouncer(ReacherConstraints.kReacherAngleSwitchDebounceTime, DebounceType.kRising);

  public Reacher() {
    this.m_extender = new WPI_TalonSRX(ArmMotorPorts.kReacherPort);
    this.m_extender.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_extender.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void reacherOut() {
    reach(ArmSpeeds.kMaxReacherSpeed);
  }

  public void reacherIn() {
    reach(-ArmSpeeds.kMaxReacherSpeed);
  }

  public void reacherInSlow() {
    reach(-ArmSpeeds.kMaxReacherSpeed * .5);
  }

  private void reach(double speed) {

    double position = getEncoderCount();
    // If the reacher is moving in and we hit limit switch, stop
    if (speed < 0 && getReacherLimitSwitch()) {
      speed = 0;
      reset();
      // If the reacher is moving in and we are close, slow down
    } else if (speed < 0 && position < ReacherConstraints.kReacherEncoderMin) {
      speed *= ReacherConstraints.kReacherSlowSpeed; // 40% power as we come in to limit switch
      // If the reacher is moving out and we hit the sensor position, stop
    } else if (speed > 0 && position >= ReacherConstraints.kReacherEncoderMax) {
      speed = 0;
    }
    m_extender.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    m_extender.set(ControlMode.PercentOutput, 0);
  }

  public int getEncoderCount() {
    return (int) m_extender.getSelectedSensorPosition();
  }

  /** Resets the integrated encoder sensor to zero */
  public void reset() {
    m_extender.setSelectedSensorPosition(0);
  }

  public double getEncoderRate() {
    return m_extender.getSelectedSensorVelocity();
  }

  public void ToggleOverride() {
    m_overrideLimit = !m_overrideLimit;
  }

  public boolean getOverride() {
    return m_overrideLimit;
  }

  public boolean getReacherLimitSwitch() {
    if (m_overrideLimit) return false;
    else return !m_reacherDebouncer.calculate(m_reacherLimitSwitch.get());
  }

  public boolean getAngleLimitSwitch() {
    if (m_overrideLimit) return false;
    else return m_angleDebouncer.calculate(m_angleLimitSwitch.get());
  }
}
