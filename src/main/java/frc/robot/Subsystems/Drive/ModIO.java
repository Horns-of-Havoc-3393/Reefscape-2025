package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.driveConstants;
import org.littletonrobotics.junction.AutoLog;

public class ModIO {

  @AutoLog
  public class ModIOIn {
    double driveVelocityRPS;
    double driveVelocityMPS;
    double steerVelocityRPS;
    double driveCurrentAmps;
    double steerCurrentAmps;
    double driveVolts;
    double steerVolts;
    Rotation2d steerPos;
  }

  TalonFX drive;
  TalonFX steer;

  StatusSignal<Double> driveVelocity;
  StatusSignal<Double> steerVelocity;
  StatusSignal<Double> driveCurrent;
  StatusSignal<Double> steerCurrent;
  StatusSignal<Double> driveVolts;
  StatusSignal<Double> steerVolts;
  StatusSignal<Double> steerPos;

  VelocityVoltage driveRequest;
  PositionVoltage steerRequest;

  public ModIO(TalonFX drive, TalonFX steer) {

    this.drive = drive;
    this.steer = steer;

    driveVelocity = drive.getVelocity();
    steerVelocity = steer.getVelocity();
    driveCurrent = drive.getStatorCurrent();
    steerCurrent = steer.getStatorCurrent();
    driveVolts = drive.getMotorVoltage();
    steerVolts = steer.getMotorVoltage();
    steerPos = steer.getPosition();
  }

  Slot0Configs dSlot0;
  Slot0Configs sSlot0;


  public void updateInputs(ModIOIn inputs) {
    BaseStatusSignal.refreshAll();

    inputs.driveVelocityRPS = driveVelocity.getValueAsDouble();
    inputs.driveVelocityMPS = driveVelocity.getValueAsDouble() / driveConstants.driveRotPerMeter;
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveVolts = driveVolts.getValueAsDouble();

    inputs.steerVelocityRPS = steerVelocity.getValueAsDouble();
    inputs.steerPos = Rotation2d.fromRotations(steerPos.getValueAsDouble());
    inputs.steerCurrentAmps = steerCurrent.getValueAsDouble();
    inputs.steerVolts = steerVolts.getValueAsDouble();
  }


  public void setDriveSpeed(double speedMPS) {
    drive.setControl(driveRequest.withVelocity(speedMPS));
  }
  public void setDriveVoltage(double volts) {drive.setControl(new VoltageOut(volts));}


  public void setSteerPos(double posDegrees) {
    steer.setControl(steerRequest.withPosition(posDegrees));
  }
  public void setSteerVoltage(double volts) {steer.setControl(new VoltageOut(volts));}


  public void setDriveVelPID(double s, double v,double p, double i, double d){
    dSlot0.kS = s;
    dSlot0.kV = v;

    dSlot0.kP = p;
    dSlot0.kI = i;
    dSlot0.kD = d;

    drive.getConfigurator().apply(dSlot0);
  }

  public void setSteerPID(double p, double i, double d) {
    sSlot0.kP = p;
    sSlot0.kI = i;
    sSlot0.kD = d;
    steer.getConfigurator().apply(sSlot0);
  }

}
