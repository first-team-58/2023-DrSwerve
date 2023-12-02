package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import java.util.Map;

public class Swerve extends SubsystemBase {
  private Boolean m_slow = true;
  private Boolean m_xStance = false;

  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.configFactoryDefault();
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry =
        new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

    configureShuffleboard();
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    if (!m_xStance) {
      ChassisSpeeds speeds =
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  translation.getX(), translation.getY(), rotation, getYaw())
              : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
      ChassisSpeeds discreteSpeeds = discretize(speeds, .02);
      SwerveModuleState[] swerveModuleStates =
          Constants.Swerve.swerveKinematics.toSwerveModuleStates(discreteSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, m_slow ? Constants.Swerve.slowSpeed : Constants.Swerve.maxSpeed);

      for (SwerveModule mod : mSwerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
      }
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void toggleSlow() {
    m_slow = !m_slow;
  }

  public void toggleXStance() {
    m_xStance = !m_xStance;
    if (m_xStance) {
      setXStance();
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public void setXStance() {
    for (SwerveModule mod : mSwerveMods) {
      mod.setModuleToX();
    }
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());
  }

  private void configureShuffleboard() {

    ShuffleboardTab tab = Shuffleboard.getTab("SwerveModules");
    int i = 0;

    for (SwerveModule mod : mSwerveMods) {
      tab.addDouble("Mod " + mod.moduleNumber + " Cancoder", () -> mod.getCanCoder().getDegrees())
          .withPosition(i, 0);
      tab.addDouble(
              "Mod " + mod.moduleNumber + " Integrated", () -> mod.getPosition().angle.getDegrees())
          .withPosition(i, 1)
          .withWidget(BuiltInWidgets.kDial)
          .withProperties(Map.of("min", 0, "max", 360));
      tab.addDouble(
              "Mod " + mod.moduleNumber + " Velocity", () -> mod.getState().speedMetersPerSecond)
          .withPosition(i, 2);
      i++;
    }

    tab.addDouble("Pigeon2 Gyroscope", () -> gyro.getYaw())
        .withPosition(4, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kGyro);

    tab.addBoolean("XStance", () -> this.m_xStance)
        .withPosition(6, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kBooleanBox);
  }

  /**
   * Discretizes a continuous-time chassis speed.
   *
   * <p>This function converts a continuous-time chassis speed into a discrete-time one such that
   * when the discrete-time chassis speed is applied for one timestep, the robot moves as if the
   * velocity components are independent (i.e., the robot moves v_x * dt along the x-axis, v_y * dt
   * along the y-axis, and omega * dt around the z-axis).
   *
   * <p>This is useful for compensating for translational skew when translating and rotating a
   * swerve drivetrain.
   *
   * @param continuousSpeeds The continuous speeds.
   * @param dtSeconds The duration of the timestep the speeds should be applied for.
   * @return Discretized ChassisSpeeds.
   */
  public ChassisSpeeds discretize(ChassisSpeeds continuousSpeeds, double dtSeconds) {
    return discretize(
        continuousSpeeds.vxMetersPerSecond,
        continuousSpeeds.vyMetersPerSecond,
        continuousSpeeds.omegaRadiansPerSecond,
        dtSeconds);
  }

  /**
   * Discretizes a continuous-time chassis speed.
   *
   * <p>This function converts a continuous-time chassis speed into a discrete-time one such that
   * when the discrete-time chassis speed is applied for one timestep, the robot moves as if the
   * velocity components are independent (i.e., the robot moves v_x * dt along the x-axis, v_y * dt
   * along the y-axis, and omega * dt around the z-axis).
   *
   * <p>This is useful for compensating for translational skew when translating and rotating a
   * swerve drivetrain.
   *
   * @param vxMetersPerSecond Forward velocity.
   * @param vyMetersPerSecond Sideways velocity.
   * @param omegaRadiansPerSecond Angular velocity.
   * @param dtSeconds The duration of the timestep the speeds should be applied for.
   * @return Discretized ChassisSpeeds.
   */
  private ChassisSpeeds discretize(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      double dtSeconds) {
    var desiredDeltaPose =
        new Pose2d(
            vxMetersPerSecond * dtSeconds,
            vyMetersPerSecond * dtSeconds,
            new Rotation2d(omegaRadiansPerSecond * dtSeconds));
    var twist = new Pose2d().log(desiredDeltaPose);
    return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
  }
}
