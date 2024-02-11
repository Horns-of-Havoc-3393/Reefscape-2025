package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveMod {

  ModIOTalon io;
  ModIOInAutoLogged inputs;

  int id;

  public SwerveMod(TalonFX drive, TalonFX steer) {
    id = drive.getDeviceID();

    io = new ModIOTalon(drive, steer);
    inputs = new ModIOInAutoLogged();

    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + drive.getDeviceID(), inputs);
  }

  public void periodic() {
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
