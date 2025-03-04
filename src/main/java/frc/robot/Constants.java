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

import com.pathplanner.lib.config.RobotConfig;

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

  public static final class autonConstants {

    public static final RobotConfig pathPlannerConfig = getPathPlannerConfig();

    private static RobotConfig getPathPlannerConfig() {
      try {
        return RobotConfig.fromGUISettings();
      } catch(Exception e) {
        e.printStackTrace();
        return null;
      }
    }

    public static final LoggedNetworkNumber transP = new LoggedNetworkNumber("/SmartDashboard/Auton/transP");
    public static final double ktransP = 0.0;
    public static final LoggedNetworkNumber transI = new LoggedNetworkNumber("/SmartDashboard/Auton/transI");
    public static final double ktransI = 0.0;
    public static final LoggedNetworkNumber transD = new LoggedNetworkNumber("/SmartDashboard/Auton/transD");
    public static final double ktransD = 0.0;

    public static final LoggedNetworkNumber rotP = new LoggedNetworkNumber("/SmartDashboard/Auton/rotP");
    public static final double krotP = 0.0;
    public static final LoggedNetworkNumber rotI = new LoggedNetworkNumber("/SmartDashboard/Auton/rotI");
    public static final double krotI = 0.0;
    public static final LoggedNetworkNumber rotD = new LoggedNetworkNumber("/SmartDashboard/Auton/rotD");
    public static final double krotD = 0.0;

  }

  public static final class driveConstants {
    public static final double currentLimit = 40; // Decrease when browning out

    // Default values for these constants should be set in initLiveConstants() below
    public static final LoggedNetworkNumber maxSpeedMPS = new LoggedNetworkNumber("/SmartDashboard/Control/LateralMaxSpeed");
    public static final double kmaxSpeedMPS = 4.0;
    public static final LoggedNetworkNumber maxRotRPS = new LoggedNetworkNumber("/SmartDashboard/Control/AngularMaxSpeed");
    public static final double kmaxRotRPS = 5.0;
    public static final LoggedNetworkNumber lateralAccelLimitMPSPS = new LoggedNetworkNumber("/SmartDashboard/Control/LateralAccel");
    public static final double klateralAccelLimitMPSPS = 4.0;
    public static final LoggedNetworkNumber rotationalAccelLimitRPSPS = new LoggedNetworkNumber("/SmartDashboard/Control/AngularAccel");
    public static final double krotationalAccelLimitRPSPS = 20.0;


    public static final double deadZone = 0.01;

    public static final double steeringRatio =
        21.42857142857143; // Rotations of motor per rotations of wheel
    public static final double wheelRadius = 1.932; // inches
    public static final double driveRatio = 5.903;

    public static final Rotation2d[] absoluteEncoderOffsets = { // Absolute encoder offsets (do no change unless swerve modules are rebuilt, or wheels are steering all over the place
      Rotation2d.fromRadians(1.890),   // To determine these values, SET THEM ALL TO 0, deploy the code and replace the 0s with "AdvantageKit/Drive/Module{#}/SteerPosAbsolute" for each module
      Rotation2d.fromRadians(-1.213),          
      Rotation2d.fromRadians(-2.163),
      Rotation2d.fromRadians(2.324)
    };

    public static final Translation2d[] offsets = { // translations of each swerve module in the order: {FrontRight, BR, BL, FL}
      new Translation2d(0.238125, -0.238125),      // When editing keep in mind that x is forward and back (+x is forward) and y is side to side (+y is *LEFT*)
      new Translation2d(-0.238125, -0.238125),
      new Translation2d(-0.238125, 0.238125),
      new Translation2d(0.238125, 0.238125)
    };

    public static final Boolean[] driveMotorInversions = {false,false,false,false}; // Set individual motor inversions (FrontRight, BR, BL, FL)

    // Default values for these constants should be set in initLiveConstants() below
    public static final LoggedNetworkNumber driveS = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveS");
    public static final double kdriveS = 0.0;
    public static final LoggedNetworkNumber driveV = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveV");
    public static final double kdriveV = 0.01;
    public static final LoggedNetworkNumber driveP = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveP");
    public static final double kdriveP = 0.03;
    public static final LoggedNetworkNumber driveI = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveI");
    public static final double kdriveI = 0.0;
    public static final LoggedNetworkNumber driveD = new LoggedNetworkNumber("/SmartDashboard/PIDs/driveD");
    public static final double kdriveD = 0.0;

    public static final LoggedNetworkNumber steerP = new LoggedNetworkNumber("/SmartDashboard/PIDs/steerP");
    public static final double ksteerP = 29;
    public static final LoggedNetworkNumber steerI = new LoggedNetworkNumber("/SmartDashboard/PIDs/steerI");
    public static final double ksteerI = 0.001;
    public static final LoggedNetworkNumber steerD = new LoggedNetworkNumber("/SmartDashboard/PIDs/steerD");
    public static final double ksteerD = 0.2;
  }


  public static final class elevatorConstants {
    public static double wristDriveRatio = 0.1; // Ratio of wrist rotations to motor rotations
    public static double rollerDriveRatio = 1; // Ratio of roller rotations to motor rotations
    public static double rollerDiameter = 2; // Diameter of rollers

    public static final LoggedNetworkNumber elvP = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvP");
    public static final double kelvP = 0.1;
    public static final LoggedNetworkNumber elvI = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvI");
    public static final double kelvI = 0.0;
    public static final LoggedNetworkNumber elvD = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvD");
    public static final double kelvD = 0.0;
    public static final LoggedNetworkNumber elvVel = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvVel");
    public static final double kelvVel = 0.0;
    public static final LoggedNetworkNumber elvAccel = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvAccel");
    public static final double kelvAccel = 0.0;
    public static final LoggedNetworkNumber elvJerk = new LoggedNetworkNumber("/SmartDashboard/PIDs/elvJerk");
    public static final double kelvJerk = 0.0;

    public static final LoggedNetworkNumber manipP = new LoggedNetworkNumber("/SmartDashboard/PIDs/manipP");
    public static final double kmanipP = 0.08;
    public static final LoggedNetworkNumber manipI = new LoggedNetworkNumber("/SmartDashboard/PIDs/manipI");
    public static final double kmanipI = 0.0;
    public static final LoggedNetworkNumber manipD = new LoggedNetworkNumber("/SmartDashboard/PIDs/manipD");
    public static final double kmanipD = 0.0;
    public static final LoggedNetworkNumber manipFF = new LoggedNetworkNumber("/SmartDashboard/PIDs/manipFF");
    public static final double kmanipFF = 0.0;
  }


  public static void initLiveConstants() {
    driveConstants.maxSpeedMPS.set(driveConstants.kmaxSpeedMPS);
    driveConstants.maxRotRPS.set(driveConstants.kmaxRotRPS);
    driveConstants.lateralAccelLimitMPSPS.set(driveConstants.klateralAccelLimitMPSPS);
    driveConstants.rotationalAccelLimitRPSPS.set(driveConstants.krotationalAccelLimitRPSPS);


    driveConstants.driveS.set(driveConstants.kdriveS);
    driveConstants.driveV.set(driveConstants.kdriveV);
    driveConstants.driveP.set(driveConstants.kdriveP);
    driveConstants.driveI.set(driveConstants.kdriveI);
    driveConstants.driveD.set(driveConstants.kdriveD);

    driveConstants.steerP.set(driveConstants.ksteerP);
    driveConstants.steerI.set(driveConstants.ksteerI);
    driveConstants.steerD.set(driveConstants.ksteerD);

    elevatorConstants.elvP.set(elevatorConstants.kelvP);
    elevatorConstants.elvI.set(elevatorConstants.kelvI);
    elevatorConstants.elvD.set(elevatorConstants.kelvD);
    elevatorConstants.elvVel.set(elevatorConstants.kelvVel);
    elevatorConstants.elvAccel.set(elevatorConstants.kelvAccel);
    elevatorConstants.elvJerk.set(elevatorConstants.kelvJerk);

    elevatorConstants.manipP.set(elevatorConstants.kmanipP);
    elevatorConstants.manipI.set(elevatorConstants.kmanipI);
    elevatorConstants.manipD.set(elevatorConstants.kmanipD);
    elevatorConstants.manipFF.set(elevatorConstants.kmanipFF);

    autonConstants.transP.set(autonConstants.ktransP);
    autonConstants.transI.set(autonConstants.ktransI);
    autonConstants.transD.set(autonConstants.ktransD);

    autonConstants.rotP.set(autonConstants.krotP);
    autonConstants.rotI.set(autonConstants.krotI);
    autonConstants.rotD.set(autonConstants.krotD);
  }


  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}

