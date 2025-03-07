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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  // private final LoggedDashboardChooser<Command> chooser =
  //     new LoggedDashboardChooser<>("Auto Choices");
  private LoggedDashboardChooser<Command> chooser;
  

  private RobotContainer robotContainer;

  private String autoName = "None"; 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to local SD card
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Initialize auto chooser

    robotContainer = new RobotContainer();


    chooser = new LoggedDashboardChooser<>("Auton/Auto Chooser", AutoBuilder.buildAutoChooser());
    //chooser.addDefaultOption("mobility", new autoCmd(robotContainer.swerve));

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
    robotContainer.swerve.zeroGyro();
    chooser.get().schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    robotContainer.swerve.setDefaultCommand(robotContainer.absCmd);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    if(!chooser.get().getName().equals(autoName) && !(chooser.get().getName().equals("InstantCommand"))) {
      System.out.println("Updating auto starting position for auto: " + chooser.get().getName());

      Optional<Pose2d> startingPose = Optional.empty();



      Iterator<PathPlannerPath> paths = null;
      PathPlannerPath startingPath = null;
      try {
        paths = PathPlannerAuto.getPathGroupFromAutoFile(chooser.get().getName()).iterator();
        startingPath = PathPlannerAuto.getPathGroupFromAutoFile(chooser.get().getName()).get(0);
      } catch (Exception e) {
        e.printStackTrace();
      }

      // Iterate through each path and add its points to the autoPoints list

      if(paths == null) {return;}

      Trajectory autoTrajectory = new Trajectory();

      while(paths.hasNext()) {
        PathPlannerPath path = paths.next();

        List<Pose2d> poses = path.getPathPoses();

        autoTrajectory = autoTrajectory.concatenate(TrajectoryGenerator.generateTrajectory(poses, new TrajectoryConfig(5.0,5.0)));
      }



      if (getFlipPath()) {
        startingPose = startingPath.flipPath().getStartingHolonomicPose();
      }else{
        startingPose = startingPath.getStartingHolonomicPose();
      }
      if(startingPose.isPresent()) {
        robotContainer.swerve.Field.getObject("startingPos").setPose(startingPose.get());
        robotContainer.swerve.Field.getObject("autoTrajectory").setTrajectory(autoTrajectory);
        // robotContainer.Field.setRobotPose(startingPose.get());
      }

      autoName = chooser.get().getName();

    }


  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private boolean getFlipPath() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}
