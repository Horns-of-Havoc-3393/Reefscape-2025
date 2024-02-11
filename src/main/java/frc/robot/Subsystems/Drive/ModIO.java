package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.modConstants;
import org.littletonrobotics.junction.AutoLog;

public class ModIO {

  @AutoLog
  public static class ModIOIn {
    static double driveVelocityRPS;
    static double driveVelocityMPS;
    static double steerVelocityRPS;
    static double driveCurrentAmps;
    static double steerCurrentAmps;
    static double driveVolts;
    static double steerVolts;
    static Rotation2d steerPos;
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

    ModIOIn.driveVelocityRPS = driveVelocity.getValueAsDouble();
    ModIOIn.driveVelocityMPS = driveVelocity.getValueAsDouble() / modConstants.driveRotPerMeter;
    ModIOIn.driveCurrentAmps = driveCurrent.getValueAsDouble();
    ModIOIn.driveVolts = driveVolts.getValueAsDouble();

    ModIOIn.steerVelocityRPS = steerVelocity.getValueAsDouble();
    ModIOIn.steerPos = Rotation2d.fromRotations(steerPos.getValueAsDouble());
    ModIOIn.steerCurrentAmps = steerCurrent.getValueAsDouble();
    ModIOIn.steerVolts = steerVolts.getValueAsDouble();
  }

  public void setDriveSpeed(double speedMPS) {
    drive.setControl(driveRequest.withVelocity(speedMPS));
  }

  public void setTurnPos(double posDegrees) {
    steer.setControl(steerRequest.withPosition(posDegrees));
  }

  public void setDriveVelPID(double s, double v,double p, double i, double d){
    dSlot0.kS = s;
    dSlot0.kV = v;

    dSlot0.kP = p;
    dSlot0.kI = i;
    dSlot0.kD = d;

    drive.getConfigurator().apply(dSlot0);
  }

  public void setTurnPID(double p, double i, double d) {
    sSlot0.kP = p;
    sSlot0.kI = i;
    sSlot0.kD = d;
    steer.getConfigurator().apply(sSlot0);
  }

}
