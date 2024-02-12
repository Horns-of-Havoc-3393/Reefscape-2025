package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.SwerveAbs;
import frc.robot.Constants.driveConstants;
import frc.robot.Positioning.PosIONavX;
import frc.robot.Subsystems.Drive.SwerveBase;

public class RobotContainer {

  TalonFX[] driveMotors = new TalonFX[4];
  TalonFX[] steerMotors = new TalonFX[4];
  CANcoder[] encoders = new CANcoder[4];
  public SwerveBase swerve;

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    deviceFactory();

    swerve =
        new SwerveBase(
            driveMotors, steerMotors, encoders, driveConstants.offsets, new PosIONavX(new AHRS()));
    configureBinds();
  }

  private void configureBinds() {
    swerve.setDefaultCommand(new SwerveAbs(swerve, controller));
  }

  private void deviceFactory() {
    for (int i = 0; i < 4; i++) {
      driveMotors[i] = new TalonFX(i + 1);
      steerMotors[i] = new TalonFX(i + 5);
      encoders[i] = new CANcoder(i + 9);
    }
  }
}
