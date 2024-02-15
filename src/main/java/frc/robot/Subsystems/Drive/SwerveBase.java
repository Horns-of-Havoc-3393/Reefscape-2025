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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveConstants;
import frc.robot.Positioning.PosIOInAutoLogged;
import frc.robot.Positioning.PosIONavX;
import org.littletonrobotics.junction.Logger;

public class SwerveBase extends SubsystemBase {
  PosIONavX posIO;
  PosIOInAutoLogged inputs;

  SwerveMod[] modules = new SwerveMod[4];

  SwerveDriveKinematics kinematics;

  DoublePublisher xVelPub;
  DoublePublisher yVelPub;

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


    posIO.updateInputs(inputs);
    Logger.processInputs("Positioning", inputs);
  }

  public void setFO(ChassisSpeeds speeds) {
    SwerveModuleState[] states =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, inputs.zGyro));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, driveConstants.maxSpeedMPS);
    Logger.recordOutput("Drive/targetStates", states);
    for (int i = 0; i < 4; i++) {
      modules[i].setSwerveState(states[i]);
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  @Override
  public void periodic() {
    posIO.updateInputs(inputs);
    Logger.processInputs("Positioning", inputs);

    for (var module : modules) {
      module.periodic();
    }

    ChassisSpeeds measuredSpeeds = kinematics.toChassisSpeeds(getStates());
    xVelPub.set(measuredSpeeds.vxMetersPerSecond);
    yVelPub.set(measuredSpeeds.vyMetersPerSecond);
    
    Logger.recordOutput("Kalman/xVelocity", measuredSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Kalman/yVelocity", measuredSpeeds.vyMetersPerSecond);

    Logger.recordOutput("ChassisAngle", inputs.zGyro);
  }
}
