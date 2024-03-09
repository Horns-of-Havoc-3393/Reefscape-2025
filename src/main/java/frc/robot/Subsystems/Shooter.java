package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  ShooterIOSparks io;
  ShooterIOInAutoLogged inputs;

  public static class angleSetpoints {
    public static Rotation2d SHOOT = Rotation2d.fromDegrees(60);
    public static Rotation2d LOAD = Rotation2d.fromDegrees(50);
    public static Rotation2d AMP = Rotation2d.fromDegrees(110);
    public static Rotation2d DRIVE = Rotation2d.fromDegrees(0);
  }

  private LoggedDashboardNumber shooterP;
  private LoggedDashboardNumber shooterI;
  private LoggedDashboardNumber shooterD;

  private LoggedDashboardNumber elevatorP;
  private LoggedDashboardNumber elevatorI;
  private LoggedDashboardNumber elevatorD;
  private LoggedDashboardNumber elevatorG;
  
  private Rotation2d angleSetpoint;
  private double speedSetpoint;

  public Shooter(
      CANSparkFlex topMotor,
      CANSparkFlex bottomMotor,
      CANSparkFlex conveyorMotor,
      CANSparkMax leftElevator,
      CANSparkMax rightElevator) {


    io = new ShooterIOSparks(topMotor,bottomMotor,conveyorMotor,leftElevator,rightElevator);
    inputs = new ShooterIOInAutoLogged();

    shooterP = new LoggedDashboardNumber("Shooter/shooterP");
    shooterI = new LoggedDashboardNumber("Shooter/shooterI");
    shooterD = new LoggedDashboardNumber("Shooter/shooterD");
    elevatorP = new LoggedDashboardNumber("Shooter/elevatorP");
    elevatorI = new LoggedDashboardNumber("Shooter/elevatorI");
    elevatorD = new LoggedDashboardNumber("Shooter/elevatorD");
    elevatorG = new LoggedDashboardNumber("Shooter/elevatorG");
   
    angleSetpoint = new Rotation2d(0);
    speedSetpoint = 0.0;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    io.setShooterPID(shooterP.get(),shooterI.get(),shooterD.get());
    io.setElevatorPID(elevatorP.get(),elevatorI.get(),elevatorD.get(),elevatorG.get()); 

    io.setElevatorAngle(angleSetpoint);
    io.setShooterSpeed(speedSetpoint, speedSetpoint);
  }
  
  public void setAngle(Rotation2d angle) {
    angleSetpoint = angle;
  }

  public void setSpeed(double speedMPS) {
    speedSetpoint = speedMPS;
  }

  public void setConveyorSpeed(double percent) {
    io.setConveyorSpeed(percent);
  }
}
