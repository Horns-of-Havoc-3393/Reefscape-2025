package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private double initialTimestamp;

  public SwerveBase(
      TalonFX[] driveMotors,
      TalonFX[] steerMotors,
      CANcoder[] encoders,
      Translation2d[] positions,
      Rotation2d[] absEncoderOffsets,
      PosIONavX posIO) {
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(positions);
    this.kinematics = kinematics;

    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveMod(driveMotors[i], steerMotors[i], encoders[i], absEncoderOffsets[i]);
    }

    this.posIO = posIO;
    inputs = new PosIOInAutoLogged();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Kalman");
    xVelPub = table.getDoubleTopic("xVelocity").publish();
    yVelPub = table.getDoubleTopic("yVelocity").publish();

    update = new LoggedNetworkBoolean("/SmartDashboard/update", false);
    zeroGyro = new LoggedNetworkBoolean("/SmartDashboard/Control/zeroGyro", false);
    publishTargetStates = new LoggedNetworkBoolean("/SmartDashboard/Control/publishTargetStates", false);

    posIO.updateInputs(inputs);
    Logger.processInputs("Positioning", inputs);

    posIO.zero();
    for (int i = 0; i < 4; i++) {
      modules[i].updatePIDs();
    }
  }

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

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }


  public SwerveModuleState[] getTargetStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getTargetState();
    }
    return states;
  }

  public void zeroGyro() {
    posIO.zero();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Timers/SwerveBasePdFreq", 1 / ((RobotController.getFPGATime() - initialTimestamp) * 0.000001));
    initialTimestamp = RobotController.getFPGATime();
    posIO.updateInputs(inputs);
    Logger.processInputs("Positioning", inputs);

    if (zeroGyro.get()) {
      posIO.zero();
    }

    for (var module : modules) {
      module.periodic();
      if (update.get()) {
        module.updatePIDs();
      }
    }

    SwerveModuleState[] states = getStates();
    SwerveModuleState[] targetStates = getTargetStates();
    Logger.recordOutput("Drive/swerveState", states);
    
    if(targetStates[0] != null){
      Logger.recordOutput("Drive/realTargetStates", targetStates);
    }
    ChassisSpeeds measuredSpeeds = kinematics.toChassisSpeeds(states);
    xVelPub.set(measuredSpeeds.vxMetersPerSecond);
    yVelPub.set(measuredSpeeds.vyMetersPerSecond);

    Logger.recordOutput("Kalman/xVelocity", measuredSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Kalman/yVelocity", measuredSpeeds.vyMetersPerSecond);

    Logger.recordOutput("ChassisAngle", inputs.zGyro);
    Logger.recordOutput(
        "Timers/SwerveBasePd", (RobotController.getFPGATime() - initialTimestamp) * 0.000001);
  }
}
