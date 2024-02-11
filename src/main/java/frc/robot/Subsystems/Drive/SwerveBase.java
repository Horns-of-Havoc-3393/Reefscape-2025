package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveConstants;
import frc.robot.Positioning.PosIOInAutoLogged;
import frc.robot.Positioning.PosIONavX;
import org.littletonrobotics.junction.Logger;

public class SwerveBase extends SubsystemBase {
  PosIONavX posIO;
  PosIOInAutoLogged inputs;

  SwerveMod[] modules;

  SwerveDriveKinematics kinematics;

  public SwerveBase(
      TalonFX[] driveMotors, TalonFX[] steerMotors, Translation2d[] positions, PosIONavX posIO) {
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(positions);
    this.kinematics = kinematics;

    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveMod(driveMotors[i], steerMotors[i]);
    }

    this.posIO = posIO;
    inputs = new PosIOInAutoLogged();

    posIO.updateInputs(inputs);
    Logger.processInputs("Positioning", inputs);
  }

  public void setFO(ChassisSpeeds speeds) {
    SwerveModuleState[] states =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, inputs.zGyro));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, driveConstants.maxSpeedMPS);
    for (int i = 0; i < 4; i++) {
      modules[i].setSwerveState(states[i]);
    }
  }

  public void periodic() {
    posIO.updateInputs(inputs);
    Logger.processInputs("Positioning", inputs);

    for (var module : modules) {
      module.periodic();
    }
  }
}
