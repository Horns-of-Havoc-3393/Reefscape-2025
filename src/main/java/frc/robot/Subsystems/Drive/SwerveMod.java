package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.driveConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class SwerveMod {

  ModIOTalon io;
  ModIOInAutoLogged inputs;

  int id;

  LoggedNetworkNumber driveS;
  LoggedNetworkNumber driveV;
  LoggedNetworkNumber driveP;
  LoggedNetworkNumber driveI;
  LoggedNetworkNumber driveD;

  LoggedNetworkNumber steerP;
  LoggedNetworkNumber steerI;
  LoggedNetworkNumber steerD;

  int encoderResets = 0;

  public SwerveMod(TalonFX drive, TalonFX steer, CANcoder absEncoder, Rotation2d absEncoderOffset) {
    id = drive.getDeviceID();

    io = new ModIOTalon(drive, steer, absEncoder, absEncoderOffset);
    inputs = new ModIOInAutoLogged();

    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + drive.getDeviceID(), inputs);

    io.setEncoderOffset((inputs.steerPosRelativePre.minus(inputs.steerPosAbsolute)));
    io.setCurrentLimit(driveConstants.currentLimit);

    driveS = new LoggedNetworkNumber("SmartDashboard/PIDs/driveS", driveConstants.driveS);
    driveS.set(driveConstants.driveS);
    driveV = new LoggedNetworkNumber("SmartDashboard/PIDs/driveV", driveConstants.driveV);
    driveV.set(driveConstants.driveV);
    driveP = new LoggedNetworkNumber("SmartDashboard/PIDs/driveP", driveConstants.driveP);
    driveP.set(driveConstants.driveP);
    driveI = new LoggedNetworkNumber("SmartDashboard/PIDs/driveI", driveConstants.driveI);
    driveI.set(driveConstants.driveI);
    driveD = new LoggedNetworkNumber("SmartDashboard/PIDs/driveD", driveConstants.driveD);
    driveD.set(driveConstants.driveD);

    steerP = new LoggedNetworkNumber("SmartDashboard/PIDs/steerP", driveConstants.driveP);
    steerP.set(driveConstants.steerP);
    steerI = new LoggedNetworkNumber("SmartDashboard/PIDs/steerI", driveConstants.driveI);
    steerI.set(driveConstants.steerI);
    steerD = new LoggedNetworkNumber("SmartDashboard/PIDs/steerD", driveConstants.driveD);
    steerD.set(driveConstants.steerD);
  }

  public void periodic() {
    double initial = RobotController.getFPGATime();
    // io.setDriveVelPID(driveS.get(), driveV.get(), driveP.get(), driveI.get(), driveD.get());
    // io.setSteerPID(steerP.get(), steerI.get(), steerD.get());
    io.updateInputs(inputs);

    if (encoderResets < 10) {
      io.setEncoderOffset((inputs.steerPosRelativePre.minus(inputs.steerPosAbsolute)));
      encoderResets++;
    }
    Logger.processInputs("Drive/Module" + id, inputs);
    Logger.recordOutput("Timers/SwerveModPd", (RobotController.getFPGATime() - initial) * 0.000001);
  }

  public void updatePIDs() {
    // io.setDriveVelPID(
    //     driveConstants.driveS,
    //     driveConstants.driveV,
    //     driveConstants.driveP,
    //     driveConstants.driveI,
    //     driveConstants.driveD);
    // io.setSteerPID(driveConstants.steerP, driveConstants.driveI, driveConstants.driveD);
    io.setDriveVelPID(driveS.get(), driveV.get(), driveP.get(), driveI.get(), driveD.get());
    io.setSteerPID(steerP.get(), steerI.get(), steerD.get());
  }

  public void setSwerveState(SwerveModuleState state) {
    Logger.recordOutput("Drive/Module" + id + "/targetSpeed", state.speedMetersPerSecond);
    Logger.recordOutput("Drive/Module" + id + "/targetAngle", state.angle);
    
    state.optimize(inputs.steerPosRelative);
    Logger.recordOutput(
        "Drive/Module" + id + "/optimizedSpeed", state.speedMetersPerSecond);
    Logger.recordOutput("Drive/Module" + id + "/optimizedAngle", state.angle);

    if (Math.abs(state.speedMetersPerSecond) < 0.1) {
      stop();
    } else {
      io.setDriveSpeed(state.speedMetersPerSecond);

      double setpoint =
          inputs.steerPosRaw + state.angle.minus(inputs.steerPosRelative).getRotations();
      io.setSteerPos(setpoint);
    }
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityMPS, inputs.steerPosRelative);
  }

  public void stop() {
    io.setDriveDutyCycle(0.0);
    io.setSteerDutyCycle(0.0);
  }
}
