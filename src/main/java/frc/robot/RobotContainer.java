package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
  SparkMax elevator1 = new SparkMax(20, MotorType.kBrushless);
  SparkMax elevator2 = new SparkMax(21, MotorType.kBrushless);
  SparkMax intake = new SparkMax(22, MotorType.kBrushless);
  SparkFlex shooter1 = new SparkFlex(24, MotorType.kBrushless);
  SparkFlex shooter2 = new SparkFlex(23, MotorType.kBrushless);
  SparkFlex conveyor = new SparkFlex(25, MotorType.kBrushless);
  public SwerveBase swerve;

  LoggedNetworkNumber shooterSpeed = new LoggedNetworkNumber("/SmartDashboard/Shooter/speed", 0.0);

  public SwerveAbs absCmd;

  private final CommandXboxController controller = new CommandXboxController(0);

  public autoCmd auto;

  long spinUp;

  public RobotContainer() {
    deviceFactory();

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
    pids.getDoubleTopic("driveS").publish().set(driveConstants.driveS);
    pids.getDoubleTopic("driveV").publish().set(driveConstants.driveV);
    pids.getDoubleTopic("driveP").publish().set(driveConstants.driveP);
    pids.getDoubleTopic("driveI").publish().set(driveConstants.driveI);
    pids.getDoubleTopic("driveD").publish().set(driveConstants.driveD);

    pids.getDoubleTopic("steerP").publish().set(driveConstants.steerP);
    pids.getDoubleTopic("steerI").publish().set(driveConstants.steerI);
    pids.getDoubleTopic("steerD").publish().set(driveConstants.steerD);

    absCmd = new SwerveAbs(swerve, controller);

  }

  private void configureBinds() {
    controller.y().onTrue(new InstantCommand(() -> {swerve.zeroGyro();}, swerve));
  }

  private void deviceFactory() {
    for (int i = 0; i < 4; i++) {
      driveMotors[i] = new TalonFX(i + 1);
      steerMotors[i] = new TalonFX(i + 5);
      encoders[i] = new CANcoder(i + 9);
    }
  }
}
