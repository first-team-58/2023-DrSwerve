package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.ArmMotorPorts;
import frc.robot.Constants.ArmConstants.ArmSpeeds;

public class Gripper extends SubsystemBase {
  private final WPI_TalonFX m_gripperMotor;

  PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  public Gripper() {
    m_gripperMotor = new WPI_TalonFX(ArmMotorPorts.kGripperPort);
    m_gripperMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void spinAtSpeed() {
    double speed = ArmSpeeds.kMaxGripperSpeed;
    m_gripperMotor.set(ControlMode.PercentOutput, speed);
  }

  public void spinOut() {
    double speed = -ArmSpeeds.kMaxGripperSpeed;
    m_gripperMotor.set(ControlMode.PercentOutput, speed);
  }

  public void spinIn() {
    double speed = ArmSpeeds.kMaxGripperSpeed;
    m_gripperMotor.set(ControlMode.PercentOutput, speed);
  }

  public void spin(double speed) {
    if (Math.abs(speed) > ArmSpeeds.kMaxGripperSpeed) {
      speed = ArmSpeeds.kMaxGripperSpeed * Math.signum(speed);
    }
    m_gripperMotor.set(speed);
  }

  public void stop() {
    m_gripperMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getAmperage() {
    return m_pdh.getCurrent(10);
  }
}
