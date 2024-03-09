package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.shooterConstants;

public class ShooterIOSparks implements ShooterIO {
  CANSparkFlex shooter1;
  CANSparkFlex shooter2;
  CANSparkFlex conveyor;
  CANSparkMax elevator1;
  CANSparkMax elevator2;

  DutyCycleEncoder absEncoder;

  DigitalInput beamBreak;

  double shooter1SetpointMPS;
  double shooter1SetpointRPS;

  double shooter2SetpointMPS;
  double shooter2SetpointRPS;

  double elevator1SetpointDeg;

  double elevator2SetpointDeg;

  PIDController shooter1PID;
  PIDController shooter2PID;

  PIDController elevator1PID;
  PIDController elevator2PID;

  double elevatorG;

  Rotation2d elevator1Offset;
  Rotation2d elevator2Offset;
  Rotation2d absOffset;

  public ShooterIOSparks(
      CANSparkFlex shooter1,
      CANSparkFlex shooter2,
      CANSparkFlex conveyor,
      CANSparkMax elevator1,
      CANSparkMax elevator2) {
    this.shooter1 = shooter1;
    this.shooter2 = shooter2;
    this.conveyor = conveyor;

    absEncoder = new DutyCycleEncoder(0);


    shooter1PID = new PIDController(shooterConstants.shooterP,shooterConstants.shooterI,shooterConstants.shooterD);
    shooter2PID = new PIDController(shooterConstants.shooterP,shooterConstants.shooterI,shooterConstants.shooterD);

    elevator1PID = new PIDController(shooterConstants.elevatorP,shooterConstants.elevatorI,shooterConstants.elevatorD);
    elevator2PID = new PIDController(shooterConstants.elevatorP,shooterConstants.elevatorI,shooterConstants.elevatorD);

    elevatorG = shooterConstants.elevatorG;
  }

  public void updateInputs(ShooterIOIn inputs) {
    inputs.shooter1DutyCycle = shooter1.getAppliedOutput();
    inputs.shooter1Volts = shooter1.getBusVoltage();
    inputs.shooter1SpeedRPS = shooter1.getAbsoluteEncoder().getVelocity();
    inputs.shooter1SpeedMPS =
        inputs.shooter1SpeedRPS * (shooterConstants.shootWheelDiameter * Math.PI);
    inputs.shooter1SetpointMPS = shooter1SetpointMPS;
    inputs.shooter1SetpointRPS = shooter1SetpointRPS;

    inputs.shooter2DutyCycle = shooter2.getAppliedOutput();
    inputs.shooter2Volts = shooter2.getBusVoltage();
    inputs.shooter2SpeedRPS = shooter2.getAbsoluteEncoder().getVelocity();
    inputs.shooter2SpeedMPS =
        inputs.shooter2SpeedRPS * (shooterConstants.shootWheelDiameter * Math.PI);
    inputs.shooter2SetpointMPS = shooter2SetpointMPS;
    inputs.shooter2SetpointRPS = shooter2SetpointRPS;

    inputs.elevator1DutyCycle = elevator1.getAppliedOutput();
    inputs.elevator1Volts = elevator1.getBusVoltage();
    inputs.elevator1SpeedRPS = elevator1.getAbsoluteEncoder().getVelocity();
    inputs.elevator1SetpointDeg = elevator1SetpointDeg;
    inputs.elevator1Position =
        Rotation2d.fromRotations(elevator1.getAbsoluteEncoder().getPosition());

    inputs.elevator2DutyCycle = elevator1.getAppliedOutput();
    inputs.elevator2Volts = elevator1.getBusVoltage();
    inputs.elevator2SpeedRPS = elevator1.getAbsoluteEncoder().getVelocity();
    inputs.elevator2SetpointDeg = elevator2SetpointDeg;
    inputs.elevator2Position =
        Rotation2d.fromRotations(elevator2.getAbsoluteEncoder().getPosition());

    inputs.shooter2DutyCycle = shooter2.getAppliedOutput();
    inputs.shooter2Volts = shooter2.getBusVoltage();
    inputs.shooter2SpeedRPS = shooter2.getAbsoluteEncoder().getVelocity();
    inputs.shooter2SpeedMPS =
        inputs.shooter2SpeedRPS * (shooterConstants.shootWheelDiameter * Math.PI);
    inputs.shooter2SetpointMPS = shooter2SetpointMPS;
    inputs.shooter2SetpointRPS = shooter2SetpointRPS;

    inputs.conveyorDutyCycle = conveyor.getAppliedOutput();
    inputs.conveyorVolts = conveyor.getBusVoltage();
    inputs.conveyorSpeedRPS = conveyor.getEncoder().getPosition();

    inputs.shooterAngleAbs = Rotation2d.fromRotations(absEncoder.getAbsolutePosition());

    inputs.beamBreak = beamBreak.get();
  }

  public void setElevatorOffsets(Rotation2d offset1, Rotation2d offset2) {
    this.elevator1Offset = offset1;
    this.elevator2Offset = offset2;
  }

  public void setShooterSpeed(double speed1MPS, double speed2MPS) {
    shooter1SetpointMPS = speed1MPS;
    shooter2SetpointMPS = speed2MPS;
    shooter1SetpointRPS = speed1MPS / (shooterConstants.shootWheelDiameter * Math.PI);
    shooter2SetpointRPS = speed2MPS / (shooterConstants.shootWheelDiameter * Math.PI);
    shooter1.set(shooter1PID.calculate(shooter1.getEncoder().getVelocity(),shooter1SetpointRPS));
    shooter2.set(shooter2PID.calculate(shooter2.getEncoder().getVelocity(),shooter2SetpointRPS));
  }

  public void setShooterDutyCycle(double percent) {
    shooter1.set(percent);
    shooter2.set(percent);
  }

  public void setShooterPID(double p, double i, double d) {
    shooter1PID.setP(p);
    shooter1PID.setI(i);
    shooter1PID.setD(d);

    shooter2PID.setP(p);
    shooter2PID.setI(i);
    shooter2PID.setD(d);
  }

  public void setElevatorAngle(Rotation2d angle) {
    double motor1Pos = angle.minus(elevator1Offset).getRotations() * (shooterConstants.elevatorConversion);
    double motor2Pos = angle.minus(elevator2Offset).getRotations() * (shooterConstants.elevatorConversion);
    elevator1.set(elevator1PID.calculate(elevator1.getEncoder().getPosition(), motor1Pos) + (Math.cos(elevator1.getEncoder().getPosition())*elevatorG));
    elevator2.set(elevator2PID.calculate(elevator2.getEncoder().getPosition(), motor2Pos) + (Math.cos(elevator2.getEncoder().getPosition())*elevatorG));
  }

  public void setElevatorDutyCycle(double percent) {
    elevator1.set(percent);
    elevator2.set(percent);
  }

  public void setElevatorPID(double p, double i, double d, double g) {
    elevator1PID.setP(p);
    elevator1PID.setI(i);
    elevator1PID.setD(d);

    elevator1PID.setP(p);
    elevator1PID.setI(i);
    elevator1PID.setD(d);

    elevatorG = g;
  }

  public void setInversions(Boolean shooter1, Boolean shooter2, Boolean conveyor) {
    this.shooter1.setInverted(shooter1);
    this.shooter2.setInverted(shooter2);
    this.conveyor.setInverted(conveyor);
  }
}
