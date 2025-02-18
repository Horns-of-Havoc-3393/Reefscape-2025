package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;

//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.SwerveAbs;
import frc.robot.Commands.autoCmd;
import frc.robot.Constants.driveConstants;
import frc.robot.Positioning.PosIONavX;
import frc.robot.Subsystems.Drive.SwerveBase;


import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RobotContainer {

  TalonFX[] driveMotors = new TalonFX[4];
  TalonFX[] steerMotors = new TalonFX[4];
  CANcoder[] encoders = new CANcoder[4];
  SparkFlex elevator1 = new SparkFlex(20, MotorType.kBrushless);
  SparkFlex elevator2 = new SparkFlex(21, MotorType.kBrushless);
  public SwerveBase swerve;

  LoggedNetworkNumber shooterSpeed = new LoggedNetworkNumber("/SmartDashboard/Shooter/speed", 0.0);

  public SwerveAbs absCmd;

  private final CommandXboxController controller = new CommandXboxController(0);

  public autoCmd auto;

  long spinUp;

  public RobotContainer() {
    deviceFactory();

    Constants.initLiveConstants();
    swerve =
        new SwerveBase(
            driveMotors,
            steerMotors,
            encoders,
            driveConstants.offsets,
            driveConstants.absoluteEncoderOffsets,
            new PosIONavX(new AHRS(AHRS.NavXComType.kMXP_SPI)));

    configureBinds();

    // auto = new autoCmd(swerve);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable pids = inst.getTable("SmartDashboard/PIDs");
    pids.getDoubleTopic("driveS").publish().set(driveConstants.driveS.get());
    pids.getDoubleTopic("driveV").publish().set(driveConstants.driveV.get());
    pids.getDoubleTopic("driveP").publish().set(driveConstants.driveP.get());
    pids.getDoubleTopic("driveI").publish().set(driveConstants.driveI.get());
    pids.getDoubleTopic("driveD").publish().set(driveConstants.driveD.get());

    pids.getDoubleTopic("steerP").publish().set(driveConstants.steerP.get());
    pids.getDoubleTopic("steerI").publish().set(driveConstants.steerI.get());
    pids.getDoubleTopic("steerD").publish().set(driveConstants.steerD.get());

    absCmd = new SwerveAbs(swerve, controller);

  }

  private void configureBinds() {
    controller.y().onTrue(new InstantCommand(() -> {swerve.zeroGyro();}, swerve));

    //swerve.setDefaultCommand(absCmd);
  }

  private void deviceFactory() {
    for (int i = 0; i < 4; i++) {
      driveMotors[i] = new TalonFX(i + 1);
      steerMotors[i] = new TalonFX(i + 5);
      encoders[i] = new CANcoder(i + 9);
    }
  }
}
