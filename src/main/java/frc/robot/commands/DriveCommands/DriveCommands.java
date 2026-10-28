// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), DriveCommandsConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks, with straight-line heading hold. A deflected
   * right stick has absolute manual rotation priority. With the right stick centered and a
   * translation input present, the gyro holds the heading the robot had when the stick was last
   * released (or at command start) — releasing the right stick never springs back toward an older
   * heading. A parked robot (no translation, no rotation input) never turns itself, and re-anchors
   * the hold target to its current heading so heading changes while parked never snap back.
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    // Heading hold reuses the angle PID already tuned for joystickDriveAtAngle.
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveCommandsConstants.ANGLE_KP,
            0.0,
            DriveCommandsConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveCommandsConstants.ANGLE_MAX_VELOCITY,
                DriveCommandsConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    Rotation2d[] holdTarget = {Rotation2d.kZero};

    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega =
                  MathUtil.applyDeadband(
                      omegaSupplier.getAsDouble(), DriveCommandsConstants.DEADBAND);

              Rotation2d rotation = drive.getRotation();
              boolean headingHoldActive;
              if (omega != 0.0) {
                // Manual rotation: absolute priority. The heading at this moment becomes the hold
                // target, so recentring the stick holds the NEW heading instead of snapping back.
                headingHoldActive = false;
                omega = Math.copySign(omega * omega, omega) * drive.getMaxAngularSpeedRadPerSec();
                holdTarget[0] = rotation;
                angleController.reset(rotation.getRadians());
              } else if (linearVelocity.getNorm() > 0.0) {
                // Translating with the right stick centered: hold the recorded heading.
                headingHoldActive = true;
                omega =
                    angleController.calculate(rotation.getRadians(), holdTarget[0].getRadians());
              } else {
                // Parked: no active turning. Re-anchor the target to the CURRENT heading so a
                // heading change while parked (a shove, a pose reset) never triggers a snap-back
                // when the driver next pushes the stick, and keep the profile fresh.
                headingHoldActive = false;
                omega = 0.0;
                holdTarget[0] = rotation;
                angleController.reset(rotation.getRadians());
              }
              Logger.recordOutput("DriveCommands/HeadingHoldActive", headingHoldActive);
              Logger.recordOutput("DriveCommands/HeadingHoldTargetDeg", holdTarget[0].getDegrees());

              // Convert to field relative speeds & send command. headingHoldActive is true
              // exactly when translation is commanded WITHOUT manual rotation — the only case
              // the startup steer-alignment gate may delay.
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()),
                  headingHoldActive);
            },
            drive)

        // A fresh (re)start — initial schedule, re-enable after disable, or the interruption the
        // B heading reset causes — must adopt the CURRENT heading, never a stale target.
        .beforeStarting(
            () -> {
              holdTarget[0] = drive.getRotation();
              angleController.reset(drive.getRotation().getRadians());
            });
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveCommandsConstants.ANGLE_KP,
            0.0,
            DriveCommandsConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveCommandsConstants.ANGLE_MAX_VELOCITY,
                DriveCommandsConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }
}
