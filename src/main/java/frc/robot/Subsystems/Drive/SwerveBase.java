package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveBase extends SubsystemBase {


    public SwerveBase(TalonFX[] driveMotors, TalonFX[] steerMotors, Translation2d[] positions) {
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(positions);


    } 
}