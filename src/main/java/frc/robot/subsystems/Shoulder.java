package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.WPI_TalonFXExtended;
import frc.robot.Constants.ArmConstants.ArmMotorPorts;
import frc.robot.Constants.ArmConstants.ArmSpeeds;
import frc.robot.Constants.ArmConstants.LimitSwitches;
import frc.robot.Constants.ArmConstants.ShoulderConstraints;
import frc.robot.Constants.Controllers;
import frc.robot.Constants.Controllers.SlewRateLimiters;

public class Shoulder extends SubsystemBase {
  /** Creates a new Shoulder. */
  private final WPI_TalonFXExtended m_shoulderLeaderMotor;

  private final WPI_TalonFXExtended m_shoulderFollowerMotor;

  private SlewRateLimiter m_moveFilter = new SlewRateLimiter(SlewRateLimiters.kSpeedSwitchSlewRate);

  DigitalInput m_frontLimitSwitch = new DigitalInput(LimitSwitches.kFrontLimitSwitch);
  DigitalInput m_backLimitSwitch = new DigitalInput(LimitSwitches.kBackLimitSwitch);

  /* Add counters to prevent mechanism from moving past switch */
  /* https://docs.wpilib.org/en/2020/docs/software/old-commandbased/commands/limit-switches-control-behavior.html */

  Counter m_frontSwitchCounter = new Counter(m_frontLimitSwitch);
  Counter m_backSwitchCounter = new Counter(m_backLimitSwitch);

  Debouncer m_debouncer =
      new Debouncer(ShoulderConstraints.kShoulderLimitSwitchDebounceTime, DebounceType.kRising);

  private double m_speed;
  private boolean m_slowMode = false;

  public Shoulder() {
    m_shoulderLeaderMotor =
        new WPI_TalonFXExtended(
            ArmMotorPorts.kShoulderLeaderPort, true, ShoulderConstraints.kShoulderGearRatio);

    m_shoulderFollowerMotor = new WPI_TalonFXExtended(ArmMotorPorts.kShoulderFollowerPort);

    m_shoulderFollowerMotor.follow(m_shoulderLeaderMotor);

    m_shoulderFollowerMotor.setNeutralMode(NeutralMode.Brake);
    m_shoulderLeaderMotor.setNeutralMode(NeutralMode.Brake);

    resetAngleToZero();

    // initalize the counters
    m_frontSwitchCounter.reset();
    m_backSwitchCounter.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void resetAngleToZero() {
    m_shoulderLeaderMotor.reset();
  }

  public void moveShoulderFrontToBack() {
    // if (getBackLimitSwitch()) {
    //   stop();
    // } else
    if (getArmAngle() > ShoulderConstraints.kShoulderBackAngleSlowMode || m_slowMode)
      move(-ArmSpeeds.kMaxOutputShoulderSlowSpeed);
    else move(-ArmSpeeds.kMaxOutputShoulderFastSpeed);
  }

  public void moveShoulderFrontToBackSlow() {
    // if (getBackLimitSwitch()) {
    //   stop();
    // } else
    if (getArmAngle() > ShoulderConstraints.kShoulderBackAngleSlowMode || m_slowMode)
      move(-ArmSpeeds.kMaxOutputShoulderSlowSpeed);
    else
      move(-ArmSpeeds.kMaxOutputShoulderFastSpeed * ShoulderConstraints.kShoulderAutoSpeedModifier);
  }

  public void moveShoulderBackToFront() {
    if (getFrontLimitSwitch()) {
      stop();
      resetAngleToZero();
    } else if (getArmAngle() < ShoulderConstraints.kShoulderFrontAngleSlowMode || m_slowMode)
      move(ArmSpeeds.kMaxOutputShoulderSlowSpeed);
    else move(ArmSpeeds.kMaxOutputShoulderFastSpeed);
  }

  public void moveShoulderBackToFrontSlow() {
    if (getFrontLimitSwitch()) {
      stop();
      resetAngleToZero();
    } else if (getArmAngle() < ShoulderConstraints.kShoulderFrontAngleSlowMode || m_slowMode)
      move(ArmSpeeds.kMaxOutputShoulderSlowSpeed);
    else
      move(ArmSpeeds.kMaxOutputShoulderFastSpeed * ShoulderConstraints.kShoulderAutoSpeedModifier);
  }

  public void move(double speed) {
    m_speed = m_moveFilter.calculate(speed);
    m_shoulderLeaderMotor.set(ControlMode.PercentOutput, m_speed);
  }

  public void stop() {
    m_shoulderLeaderMotor.set(ControlMode.PercentOutput, 0);
  }

  public void ToggleSlowMode() {
    m_slowMode = !m_slowMode;
    if (this.m_slowMode)
      Controllers.kOperatorController.setRumble(RumbleType.kBothRumble, Controllers.kRumbleValue);
    else {
      Controllers.kOperatorController.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  public double getArmAngle() {
    return m_shoulderLeaderMotor.getInvertedAngle();
  }

  public double getShoulderSpeed() {
    return m_speed;
  }

  public boolean getFrontLimitSwitch() {
    // So if currently false the signal must go true for at least .1 seconds
    // before being read as a False signal.
    return m_debouncer.calculate(m_frontLimitSwitch.get());
  }

  public double getFrontLimitSwitchCount() {
    return m_frontSwitchCounter.get();
  }

  public double getBackLimitSwitchCount() {
    return m_backSwitchCounter.get();
  }

  public boolean getBackLimitSwitch() {
    return m_backLimitSwitch.get();
  }

  public boolean getSlowMode() {
    return this.m_slowMode;
  }

  public double getEncoderCount() {
    return this.m_shoulderLeaderMotor.getSelectedSensorPosition();
  }
}
