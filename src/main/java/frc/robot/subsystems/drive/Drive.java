// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.MapleSimArena;
import frc.robot.util.DashboardControls;
import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public static Drive mInstance;

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final BaseStatusSignal[] statusSignals;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private final StartupAlignGate startupAlignGate = new StartupAlignGate();
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  // Field2d sendable so dashboards (Elastic) can show the robot pose on a field widget;
  // AdvantageKit's Odometry/Robot output is a struct that Elastic's field widget can't read.
  private final Field2d dashboardField = new Field2d();
  private int dashboardPublishCounter = 0;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    var statusSignalList = new ArrayList<BaseStatusSignal>();
    Collections.addAll(statusSignalList, gyroIO.getStatusSignals());
    for (var module : modules) {
      Collections.addAll(statusSignalList, module.getStatusSignals());
    }
    statusSignals = statusSignalList.toArray(BaseStatusSignal[]::new);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    SmartDashboard.putData("Field", dashboardField);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    try {
      // Phoenix documents a single bulk refresh as faster than refreshing signals separately.
      // Suppress DS error spam here; the per-device connection alerts below report failures.
      if (statusSignals.length > 0) {
        BaseStatusSignal.refreshAll(false, statusSignals);
      }
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      // The alignment gate must not keep pre-disable state across the disable: a re-enabled
      // held stick has to be judged against the CURRENT (rested) robot, not the mid-drive past.
      startupAlignGate.reset();
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    if (Constants.currentMode != Mode.SIM) {
      updateOdometry();
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // Field2d/NetworkTables serialization showed 20-40 ms spikes in the RIO trace. A 10 Hz field
    // display is visually smooth while leaving the 50 Hz control and AdvantageKit data untouched.
    if (++dashboardPublishCounter >= 5) {
      dashboardPublishCounter = 0;
      dashboardField.setRobotPose(getPose());
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, true);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   * @param allowStartupAlign false while the caller commands manual rotation (or otherwise must not
   *     be delayed by the startup steer-alignment gate)
   */
  public void runVelocity(ChassisSpeeds speeds, boolean allowStartupAlign) {
    // Apply the dashboard Drive/Turn speed limits (no-op in autonomous)
    speeds = DashboardControls.applyDriveLimits(speeds);

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Startup steer alignment: a TRANSLATION command that begins from measured rest (and is not
    // flagged manual rotation) holds the module drive output at zero until every steer angle
    // reaches its (optimize-aware) setpoint, so a full-speed start cannot drive the chassis
    // while the wheels still point somewhere else. Rotation-only commands never arm, a
    // translation edge while still rolling never re-arms (direction changes at speed keep the
    // ordinary cosine-scaled steering), and the hold is bounded by
    // StartupAlignGate.ALIGN_TIMEOUT_SEC. A zero-speed command targets each module's last
    // COMMANDED heading (the WPILib kinematics cache) at zero drive speed, so wheels already
    // there simply hold their direction while parked.
    boolean translationCommanded =
        Math.abs(discreteSpeeds.vxMetersPerSecond) > 1e-6
            || Math.abs(discreteSpeeds.vyMetersPerSecond) > 1e-6;
    boolean allSteerAnglesAligned = true;
    for (int i = 0; i < 4; i++) {
      if (StartupAlignGate.optimizedSteerErrorRad(setpointStates[i].angle, modules[i].getAngle())
          > StartupAlignGate.ALIGN_TOLERANCE_RAD) {
        allSteerAnglesAligned = false;
        break;
      }
    }
    ChassisSpeeds measuredSpeeds = getChassisSpeeds();
    boolean measuredAtRest =
        Math.abs(measuredSpeeds.vxMetersPerSecond) < StartupAlignGate.REST_LINEAR_SPEED_MPS
            && Math.abs(measuredSpeeds.vyMetersPerSecond) < StartupAlignGate.REST_LINEAR_SPEED_MPS
            && Math.abs(measuredSpeeds.omegaRadiansPerSecond)
                < StartupAlignGate.REST_ANGULAR_SPEED_RADPS;
    boolean holdStartupDrive =
        startupAlignGate.shouldHoldDrive(
            Logger.getTimestamp() * 1e-6,
            translationCommanded,
            allowStartupAlign,
            measuredAtRest,
            allSteerAnglesAligned);
    Logger.recordOutput("Drive/StartupAlignHold", holdStartupDrive);
    if (holdStartupDrive) {
      for (int i = 0; i < 4; i++) {
        setpointStates[i].speedMetersPerSecond = 0.0;
      }
    }

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Runs the drive at the desired velocity, rotating around a custom center of rotation instead of
   * the robot center. Used for corner-pivot maneuvers.
   *
   * @param speeds Desired chassis speeds (typically only omega is nonzero)
   * @param centerOfRotation Center of rotation in robot frame (meters)
   */
  public void runVelocityWithCenterOfRotation(
      ChassisSpeeds speeds, Translation2d centerOfRotation) {
    speeds = DashboardControls.applyDriveLimits(speeds);
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates =
        kinematics.toSwerveModuleStates(discreteSpeeds, centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Resets the current odometry pose. Keeps rawGyroRotation untouched so the pose estimator records
   * the offset between the physical gyro reading and the new heading; overwriting it would zero the
   * offset and let the next raw gyro sample revert the reset.
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    if (Constants.currentMode == Mode.SIM) {
      rawGyroRotation = pose.getRotation();
      MapleSimArena.getInstance().setRobotPose(pose);
    }
  }

  private void updateOdometry() {
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  /**
   * Resets odometry to match the maple-sim physics pose. Does not move the physics body — use
   * {@link #setPose(Pose2d)} when teleporting both.
   */
  public void setPoseFromSimulation(Pose2d pose) {
    rawGyroRotation = pose.getRotation();
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
