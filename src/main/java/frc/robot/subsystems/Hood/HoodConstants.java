// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.Hood;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.io.MotorIOPhoenix6.ClosedLoopOutput;
import frc.lib.io.MotorIOPhoenix6.MotorIOPhoenix6Config;
import frc.lib.io.MotorIOPhoenix6.NeutralMode;
import frc.lib.io.MotorIOSim.MotorIOSimConfig;

public final class HoodConstants {
  public static final String CAN_BUS = "rio";

  public static final int Hood_ID = 46;

  public static final double Hood_ROTOR_TO_MECHANISM_RATIO = 56.3636363636363636366363636;

  // Mechanism position (rotations) the encoder is reset to once the hood drives into its hard stop.
  public static final double HoodZeroPosition = 0.0;

  // Hood Angle (%) range in mechanism degrees (position rot * 360). 0% is the zeroed hard stop
  // (fully lowered, steepest shot); 100% is 45 deg, raised 2026-10-26 by operator request so the
  // hood can nearly fully deploy. This is above the calibrated shot tables (their max was the 25
  // deg ferry table), and the real mechanical travel is still unmeasured — verify 45 deg against
  // the physical stop. No upper soft limit is configured on the Talon, so this is the enforced
  // ceiling.
  public static final double MinAngleDeg = HoodZeroPosition * 360.0;
  public static final double MaxAngleDeg = 45.0;

  public static final MotorIOPhoenix6Config Hood_CONFIG =
      new MotorIOPhoenix6Config(Hood_ID, CAN_BUS)
          .withRotorToMechanismRatio(Hood_ROTOR_TO_MECHANISM_RATIO)
          .withInverted(true)
          .withNeutralMode(NeutralMode.BRAKE)
          .withCurrentLimits(30.0, 60.0)
          .withSlot0(1000., 0.0, 200, 0.0, 0.0, 0.0, 0.0)
          .withMotionMagic(2, 8, 0)
          .withClosedLoopOutput(ClosedLoopOutput.TORQUE_CURRENT_FOC);

  public static final MotorIOSimConfig Hood_SIM_CONFIG =
      new MotorIOSimConfig()
          .withGearbox(DCMotor.getKrakenX60Foc(2))
          .withGearing(Hood_ROTOR_TO_MECHANISM_RATIO)
          .withMomentOfInertia(0.003)
          .withVelocityGains(0.12, 0.0)
          .withInstantSetpoint(true);

  private HoodConstants() {}
}
