package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.modConstants;


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

    public ModIO(TalonFX drive, TalonFX steer) {

        this.drive=drive;
        this.steer=steer;

        driveVelocity = drive.getVelocity();
        steerVelocity = steer.getVelocity();
        driveCurrent = drive.getStatorCurrent();
        steerCurrent = steer.getStatorCurrent();
        driveVolts = drive.getMotorVoltage();
        steerVolts = steer.getMotorVoltage();
        steerPos = steer.getPosition();
        
    }


    public void updateInputs(ModIOIn inputs) {
        BaseStatusSignal.refreshAll();

        ModIOIn.driveVelocityRPS = driveVelocity.getValueAsDouble();
        ModIOIn.driveVelocityMPS = driveVelocity.getValueAsDouble()/modConstants.driveRotPerMeter;
        ModIOIn.driveCurrentAmps = driveCurrent.getValueAsDouble();
        ModIOIn.driveVolts = driveVolts.getValueAsDouble();

        ModIOIn.steerVelocityRPS = steerVelocity.getValueAsDouble();
        ModIOIn.steerPos = Rotation2d.fromRotations(steerPos.getValueAsDouble());
        ModIOIn.steerCurrentAmps = steerCurrent.getValueAsDouble();
        ModIOIn.steerVolts = steerVolts.getValueAsDouble();
    }
}