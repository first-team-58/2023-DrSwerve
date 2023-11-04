package frc.lib.util;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public final class WPI_TalonFXExtended extends WPI_TalonFX {
  private final int kEncoderCPR = 2048;
  private final boolean m_useIntegratedSensor;
  private double kSensorGearRatio = 0;
  private double kWheelDiameterMeters = 0;
  private double kWheelCircumferenceMeters = Math.PI * kWheelDiameterMeters;
  private double kInvertAngle = -1;

  /**
   * Constructor for motor controller
   *
   * @param deviceNumber device ID of motor controller
   */
  public WPI_TalonFXExtended(int deviceNumber) {
    super(deviceNumber);
    this.m_useIntegratedSensor = false;
    config();
  }

  /**
   * Constructor for motor controller
   *
   * @param deviceNumber device ID of motor controller
   * @param useIntegratedSensor whether to use integrated sensor
   * @param gearRatio gear ratio between mechanism and sensor
   */
  public WPI_TalonFXExtended(int deviceNumber, boolean useIntegratedSensor, double gearRatio) {
    super(deviceNumber);
    this.m_useIntegratedSensor = true;
    this.kSensorGearRatio = gearRatio;
    config();
  }

  /**
   * Constructor for motor controller
   *
   * @param deviceNumber device ID of motor controller
   * @param useIntegratedSensor whether to use integrated sensor
   * @param gearRatio gear ratio between mechanism and sensor
   * @param wheelDiamterMeters drive base use only for tracking distance in meters
   */
  public WPI_TalonFXExtended(
      int deviceNumber, boolean useIntegratedSensor, double gearRatio, double wheelDiameterMeters) {
    super(deviceNumber);
    this.m_useIntegratedSensor = true;
    this.kSensorGearRatio = gearRatio;
    this.kWheelDiameterMeters = wheelDiameterMeters;
    config();
  }

  /** Resets the integrated encoder sensor to zero */
  public void reset() {
    this.setSelectedSensorPosition(0);
  }

  /* ODOMETRY */
  public double getTalonVelocity() {
    return metersPerSecToEdgesPerDecisec(this.getSelectedSensorVelocity());
  }

  /** Team 7028 conversion talonfx to wpi and vice-versa */
  /**
   * Converts from encoder edges to meters.
   *
   * @param edges encoder edges to convert
   * @return meters
   */
  public double edgesToMeters(double edges) {
    return (kWheelCircumferenceMeters / (kEncoderCPR * kSensorGearRatio)) * edges;
  }

  /**
   * Converts from encoder edges per 100 milliseconds to meters per second.
   *
   * @param stepsPerDecisec edges per decisecond
   * @return meters per second
   */
  public double edgesPerDecisecToMetersPerSec(double stepsPerDecisec) {
    return edgesToMeters(stepsPerDecisec / .1d);
  }

  /**
   * Converts from meters to encoder edges.
   *
   * @param meters meters
   * @return encoder edges
   */
  public double metersToEdges(double meters) {
    return (meters / kWheelCircumferenceMeters) * (kEncoderCPR * kSensorGearRatio);
  }
  /**
   * Converts from meters per second to encoder edges per 100 milliseconds.
   *
   * @param metersPerSec meters per second
   * @return encoder edges per decisecond
   */
  public double metersPerSecToEdgesPerDecisec(double metersPerSec) {
    return metersToEdges(metersPerSec) * .1d;
  }

  /**
   * Converts TalonFX native units to distance
   *
   * @return distance in meters reported by encoder
   */
  public double getDistance() {
    return nativeUnitsToDistanceMeters(this.getSelectedSensorPosition());
  }

  /**
   * Converts TalonFX native units to velocity Needs research as Talons run on 10ms loop, while
   * roborio is 20ms loop
   *
   * @return velocity in meters per second reported by encoder
   */
  public double getRate() {
    return velocityMetersPerSecond(this.getSelectedSensorVelocity());
  }

  /**
   * Converts TalonFX native units to angle. Useful for arm mechanisms
   *
   * @return angle in degrees
   */
  public double getAngle() {
    double motorRotation = this.getSelectedSensorPosition() / kEncoderCPR;
    double angleRotation = motorRotation * kSensorGearRatio;
    double angle = angleRotation / 360;
    return angle;
  }

  /**
   * Inverts the Angle direction if the motors are spinning backwards
   *
   * @return angle in degrees
   */
  public double getInvertedAngle() {
    return getAngle() * kInvertAngle;
  }

  private double velocityMetersPerSecond(double rate) {
    // current sensor rate from falcon is 10 ms.
    double velocity = rate * (10.0 / kEncoderCPR) * kWheelDiameterMeters;
    return velocity;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / kEncoderCPR;
    double wheelRotations = motorRotations / kSensorGearRatio;
    double positionMeters = wheelRotations * (Math.PI * kWheelDiameterMeters);
    return positionMeters;
  }

  private void config() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    this.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
    this.configNeutralDeadband(0.001);

    /* Config integrated encoder */
    if (this.m_useIntegratedSensor) {
      System.out.println("Setting integrated sensor for encoder use");
      this.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    }
  }
}
