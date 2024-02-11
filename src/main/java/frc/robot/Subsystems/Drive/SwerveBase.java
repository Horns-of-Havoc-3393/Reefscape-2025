package frc.robot.Subsystems.Drive;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PosIO;
import frc.robot.PosIOInAutoLogged;
import frc.robot.Constants.driveConstants;

public class SwerveBase extends SubsystemBase {
    PosIO posIO;
    PosIOInAutoLogged inputs;

    SwerveMod[] modules;

    SwerveDriveKinematics kinematics;

    public SwerveBase(TalonFX[] driveMotors, TalonFX[] steerMotors, Translation2d[] positions, PosIO posIO) {
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(positions);
        this.kinematics = kinematics;

        for(int i=0; i<4; i++){
            modules[i] = new SwerveMod(driveMotors[i], steerMotors[i]);
        }

        this.posIO = posIO;
        inputs = new PosIOInAutoLogged();

        posIO.updateInputs(inputs);
        Logger.processInputs("Positioning", inputs);
    }

    public void setFO(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, inputs.zGyro));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, driveConstants.maxSpeedMPS);
        for(int i=0; i<4; i++){
            modules[i].setSwerveState(states[i]);
        }

    }

    public void periodic() {
        posIO.updateInputs(inputs);
        Logger.processInputs("Positioning", inputs);

        for(var module : modules) {
            module.periodic();
        }
    }

}
