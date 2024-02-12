package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.SwerveAbs;
import frc.robot.Constants.driveConstants;
import frc.robot.Positioning.PosIONavX;
import frc.robot.Subsystems.Drive.SwerveBase;

public class RobotContainer {

  TalonFX[] driveMotors = new TalonFX[4];
  TalonFX[] steerMotors = new TalonFX[4];
  SwerveBase swerve;

  private final CommandXboxController controller = new CommandXboxController(0);

  RobotContainer() {
    motorFactory();
    publishPIDs();

    swerve =
        new SwerveBase(driveMotors, steerMotors, driveConstants.offsets, new PosIONavX(new AHRS()));
    configureBinds();
  }

  private void configureBinds() {
    swerve.setDefaultCommand(
        new SwerveAbs(
            swerve,
            () -> controller.getLeftX(),
            () -> controller.getLeftY(),
            () -> controller.getRightX()));
  }

  private void motorFactory() {
    for (int i = 0; i < 4; i++) {
      driveMotors[i] = new TalonFX(i + 1);
      steerMotors[i] = new TalonFX(i + 5);
    }
  }
  private void publishPIDs() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("/PIDs");
    table.getDoubleTopic("driveS").publish().set(driveConstants.driveS);
    table.getDoubleTopic("driveV").publish().set(driveConstants.driveV);
    table.getDoubleTopic("driveP").publish().set(driveConstants.driveP);
    table.getDoubleTopic("driveI").publish().set(driveConstants.driveI);
    table.getDoubleTopic("driveD").publish().set(driveConstants.driveD);
    
    table.getDoubleTopic("driveP").publish().set(driveConstants.steerP);
    table.getDoubleTopic("driveI").publish().set(driveConstants.steerI);
    table.getDoubleTopic("driveD").publish().set(driveConstants.steerD);
  }
}
