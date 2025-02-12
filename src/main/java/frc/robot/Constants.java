// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class driveConstants {
    public static final double currentLimit = 40; // Decrease when browning out

    // Default values for these constants should be set in initLiveConstants() below
    public static final LoggedNetworkNumber maxSpeedMPS = new LoggedNetworkNumber("/SmartDashboard/Control/LateralMaxSpeed");
    public static final LoggedNetworkNumber maxRotRPS = new LoggedNetworkNumber("/SmartDashboard/Control/AngularMaxSpeed");
    public static final LoggedNetworkNumber lateralAccelLimitMPSPS = new LoggedNetworkNumber("/SmartDashboard/Control/LateralAccel");
    public static final LoggedNetworkNumber rotationalAccelLimitRPSPS = new LoggedNetworkNumber("/SmartDasboard/Control/AngularAccel");


    public static final double deadZone = 0.01;

    public static final double steeringRatio =
        21.42857142857143; // Rotations of motor per rotations of wheel
    public static final double wheelRadius = 1.932; // inches
    public static final double driveRatio = 5.903;

    public static final Rotation2d[] absoluteEncoderOffsets = {
      Rotation2d.fromRadians(0.332),
      Rotation2d.fromRadians(-0.664),
      Rotation2d.fromRadians(1.921),
      Rotation2d.fromRadians(-0.823)
    };

    public static final Translation2d[] offsets = {
      new Translation2d(0.238125, -0.238125),
      new Translation2d(-0.238125, -0.238125),
      new Translation2d(-0.238125, 0.238125),
      new Translation2d(0.238125, 0.238125)
    };

    // Default values for these constants should be set in initLiveConstants() below
    public static final LoggedNetworkNumber driveS = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveS");
    public static final LoggedNetworkNumber driveV = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveV");
    public static final LoggedNetworkNumber driveP = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveP");
    public static final LoggedNetworkNumber driveI = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveI");
    public static final LoggedNetworkNumber driveD = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveD");

    public static final LoggedNetworkNumber steerP = new LoggedNetworkNumber("/SmartDashboard/PIDs/steerP");
    public static final LoggedNetworkNumber steerI = new LoggedNetworkNumber("/SmartDashboard/PIDs/steerI");
    public static final LoggedNetworkNumber steerD = new LoggedNetworkNumber("/SmartDashboard/PIDs/steerD");
  }


  public static void initLiveConstants() {
    driveConstants.maxSpeedMPS.set(0.6);
    driveConstants.maxRotRPS.set(5);
    driveConstants.lateralAccelLimitMPSPS.set(4);
    driveConstants.rotationalAccelLimitRPSPS.set(20);


    driveConstants.driveS.set(0);
    driveConstants.driveV.set(0.01);
    driveConstants.driveP.set(0.03);
    driveConstants.driveI.set(0.0);
    driveConstants.driveD.set(0.0);

    driveConstants.steerP.set(29);
    driveConstants.steerI.set(0.001);
    driveConstants.steerD.set(0.2);
  }


  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}

