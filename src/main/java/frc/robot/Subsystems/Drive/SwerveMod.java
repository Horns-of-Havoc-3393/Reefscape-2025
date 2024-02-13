package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.driveConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class SwerveMod {

  ModIOTalon io;
  ModIOInAutoLogged inputs;

  int id;

  LoggedDashboardNumber driveS;
  LoggedDashboardNumber driveV;
  LoggedDashboardNumber driveP;
  LoggedDashboardNumber driveI;
  LoggedDashboardNumber driveD;

  LoggedDashboardNumber steerP;
  LoggedDashboardNumber steerI;
  LoggedDashboardNumber steerD;

  public SwerveMod(TalonFX drive, TalonFX steer, CANcoder absEncoder, Rotation2d absEncoderOffset) {
    id = drive.getDeviceID();

    io = new ModIOTalon(drive, steer, absEncoder, absEncoderOffset);
    inputs = new ModIOInAutoLogged();

    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + drive.getDeviceID(), inputs);

    io.setEncoderOffset((inputs.steerPosRelative.minus(inputs.steerPosAbsolute)));

    driveS = new LoggedDashboardNumber("PIDs/driveS", driveConstants.driveS);
    driveV = new LoggedDashboardNumber("PIDs/driveV", driveConstants.driveV);
    driveP = new LoggedDashboardNumber("PIDs/driveP", driveConstants.driveP);
    driveI = new LoggedDashboardNumber("PIDs/driveI", driveConstants.driveI);
    driveD = new LoggedDashboardNumber("PIDs/driveD", driveConstants.driveD);

    steerP = new LoggedDashboardNumber("PIDs/steerP", driveConstants.driveP);
    steerI = new LoggedDashboardNumber("PIDs/steerI", driveConstants.driveI);
    steerD = new LoggedDashboardNumber("PIDs/steerD", driveConstants.driveD);
  }

  public void periodic() {
    io.setDriveVelPID(driveS.get(), driveV.get(), driveP.get(), driveI.get(), driveD.get());
    io.setSteerPID(steerP.get(), steerI.get(), steerD.get());
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + id, inputs);
  }

  public void setSwerveState(SwerveModuleState state) {
    Logger.recordOutput("Drive/Module" + id + "/targetSpeed", state.speedMetersPerSecond);
    Logger.recordOutput("Drive/Module" + id + "/targetAngle", state.angle);
    SwerveModuleState.optimize(state, inputs.steerPosRelative);

    io.setDriveSpeed(state.speedMetersPerSecond);
    io.setSteerPos(state.angle.getDegrees());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityMPS, inputs.steerPosRelative);
  }

  public void stop() {
    io.setDriveVoltage(0.0);
    io.setSteerVoltage(0.0);
  }
}
