package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveConstants;
import frc.robot.Positioning.PosIOInAutoLogged;
import frc.robot.Positioning.PosIONavX;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class SwerveBase extends SubsystemBase {
  PosIONavX posIO;
  PosIOInAutoLogged inputs;

  SwerveMod[] modules = new SwerveMod[4];

  SwerveDriveKinematics kinematics;

  DoublePublisher xVelPub;
  DoublePublisher yVelPub;

  LoggedNetworkBoolean update;
  LoggedNetworkBoolean zeroGyro;
  LoggedNetworkBoolean publishTargetStates;

  SwerveDriveOdometry odometry;

  int PIDUpdates = 0;

  private double initialTimestamp;

  public SwerveBase(
      TalonFX[] driveMotors,
      TalonFX[] steerMotors,
      CANcoder[] encoders,
      Translation2d[] positions,
      Rotation2d[] absEncoderOffsets,
      PosIONavX posIO)
    {
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(positions);
    this.kinematics = kinematics;

    // instantiate swerve mod classes
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveMod(driveMotors[i], steerMotors[i], encoders[i], absEncoderOffsets[i]);
    }



    this.posIO = posIO;
    inputs = new PosIOInAutoLogged();


    // debug telemetry stuff
    update = new LoggedNetworkBoolean("/SmartDashboard/update", false);
    zeroGyro = new LoggedNetworkBoolean("/SmartDashboard/Control/zeroGyro", false);
    publishTargetStates = new LoggedNetworkBoolean("/SmartDashboard/Control/publishTargetStates", false);



    posIO.updateInputs(inputs);
    Logger.processInputs("Positioning", inputs);


    // Zero gyro and set PIDs of each module
    posIO.zero();
    for (int i = 0; i < 4; i++) {
      modules[i].updatePIDs();
    }


    // init odometry
    odometry = new SwerveDriveOdometry(
      new SwerveDriveKinematics(driveConstants.offsets),
      inputs.zGyro,
      getPositions());
  }


  // Set Field Oriented: moves the robot according to the field-oriented "speeds" ChassisSpeeds object
  public void setFO(ChassisSpeeds speeds, double lateralMaxSpeed) {
    double initial = RobotController.getFPGATime();

    SwerveModuleState[] states =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, inputs.zGyro));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, lateralMaxSpeed);

    Logger.recordOutput("Drive/targetStates", states);

    for (int i = 0; i < 4; i++) {
      modules[i].setSwerveState(states[i]);
    }
    Logger.recordOutput("Timers/SwerveBaseSetFO", (RobotController.getFPGATime() - initial) * 0.000001);
  }


  // Get the current state of each swerve module
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }


  // Get the current position of each swerve module (uses distance traveled by drive motor rather than current velocity)
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }


  // Get the current target states for each module (for logging/debug purposes)
  public SwerveModuleState[] getTargetStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getTargetState();
    }
    return states;
  }


  // zero the gryo
  public void zeroGyro() {
    posIO.zero();
  }

  public void updatePIDs() {
    for (int i=0; i<4; i++) {
      modules[i].updatePIDs();
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Timers/SwerveBasePdFreq", 1 / ((RobotController.getFPGATime() - initialTimestamp) * 0.000001));
    initialTimestamp = RobotController.getFPGATime();



    posIO.updateInputs(inputs);
    Logger.processInputs("Positioning", inputs);


    // debug stuff
    if (zeroGyro.get()) {
      posIO.zero();
    }

    for (var module : modules) {
      module.periodic();
      if (update.get() || PIDUpdates < 10) {
        System.out.println("Update base pids");
        module.updatePIDs();
        PIDUpdates++;
      }
    }



    // logging/debug
    SwerveModuleState[] states = getStates();
    SwerveModuleState[] targetStates = getTargetStates();
    Logger.recordOutput("Drive/swerveState", states);
    
    if(targetStates[0] != null){
      Logger.recordOutput("Drive/realTargetStates", targetStates);
    }
    ChassisSpeeds measuredSpeeds = kinematics.toChassisSpeeds(states);
    xVelPub.set(measuredSpeeds.vxMetersPerSecond);
    yVelPub.set(measuredSpeeds.vyMetersPerSecond);



    // step odometry
    odometry.update(inputs.zGyro, getPositions());


    Logger.recordOutput("Kalman/xVelocity", measuredSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Kalman/yVelocity", measuredSpeeds.vyMetersPerSecond);

    Logger.recordOutput("ChassisAngle", inputs.zGyro);
    Logger.recordOutput(
        "Timers/SwerveBasePd", (RobotController.getFPGATime() - initialTimestamp) * 0.000001);
  }
}
