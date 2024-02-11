package frc.robot.Subsystems.Drive;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveMod {

    ModIO io;
    ModIOInAutoLogged inputs;

    int id;

    public SwerveMod(TalonFX drive, TalonFX steer) {
        id = drive.getDeviceID();

        ModIO io = new ModIO(drive, steer);
        ModIOInAutoLogged inputs = new ModIOInAutoLogged();

        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + drive.getDeviceID(), inputs);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + id, inputs);
    }

    public void setSwerveState(SwerveModuleState state){
        SwerveModuleState.optimize(state, inputs.steerPos);

        io.setDriveSpeed(state.speedMetersPerSecond);
        io.setSteerPos(state.angle.getDegrees());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.driveVelocityMPS, inputs.steerPos);
    }

    public void stop(){
        io.setDriveVoltage(0.0);
        io.setSteerVoltage(0.0);
    }
}
