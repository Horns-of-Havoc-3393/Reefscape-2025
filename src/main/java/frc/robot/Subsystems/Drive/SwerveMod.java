package frc.robot.Subsystems.Drive;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveMod {

    public SwerveMod(TalonFX drive, TalonFX steer) {
        ModIO io = new ModIO(drive, steer);
        ModIOInAutoLogged inputs = new ModIOInAutoLogged();

        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + drive.getDeviceID(), inputs);
    }
}
