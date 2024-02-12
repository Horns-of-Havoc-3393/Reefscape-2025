package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

  public SwerveMod(TalonFX drive, TalonFX steer) {
    id = drive.getDeviceID();

    io = new ModIOTalon(drive, steer);
    inputs = new ModIOInAutoLogged();

    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + drive.getDeviceID(), inputs);

    driveS = new LoggedDashboardNumber("PIDs/driveS");
    driveV = new LoggedDashboardNumber("PIDs/driveV");
    driveP = new LoggedDashboardNumber("PIDs/driveP");
    driveI = new LoggedDashboardNumber("PIDs/driveI");
    driveD = new LoggedDashboardNumber("PIDs/driveD");

    steerP = new LoggedDashboardNumber("PIDs/steerP");
    steerI = new LoggedDashboardNumber("PIDs/steerI");
    steerD = new LoggedDashboardNumber("PIDs/steerD");
  }

  public void periodic() {
    io.setDriveVelPID(driveS.get(),driveV.get(),driveP.get(),driveI.get(),driveD.get());
    io.setSteerPID(steerP.get(),steerI.get(),steerD.get());
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + id, inputs);
  }

  public void setSwerveState(SwerveModuleState state) {
    SwerveModuleState.optimize(state, inputs.steerPos);

    io.setDriveSpeed(state.speedMetersPerSecond);
    io.setSteerPos(state.angle.getDegrees());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityMPS, inputs.steerPos);
  }

  public void stop() {
    io.setDriveVoltage(0.0);
    io.setSteerVoltage(0.0);
  }
}
