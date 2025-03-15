package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.elevatorConstants;

public class ManipulatorIONEO implements ManipulatorIO {

    SparkMax wristMotor;
    SparkMax rollerMotor;
    RelativeEncoder absoluteEncoder;

    SparkMaxConfig wristConfig;
    SparkMaxConfig rollerConfig;

    ClosedLoopConfig wristPID;
    ClosedLoopConfig rollerPID;

    DigitalInput beamBreakBack = new DigitalInput(1);
    DigitalInput beamBreakFront = new DigitalInput(0);

    double wristOffset;

    
    ManipulatorIONEO(SparkMax wristMotor, SparkMax rollerMotor) {
        this.wristMotor = wristMotor;
        this.rollerMotor = rollerMotor;
        absoluteEncoder = wristMotor.getAlternateEncoder();

        // Motor configs ------------------------------------------------
        wristConfig = new SparkMaxConfig();
        wristConfig.alternateEncoder.setSparkMaxDataPortConfig();
        wristConfig.alternateEncoder.countsPerRevolution(8192);
        wristConfig.softLimit.forwardSoftLimitEnabled(true);
        wristConfig.softLimit.reverseSoftLimitEnabled(true);
        wristConfig.softLimit.forwardSoftLimit(0);
        wristConfig.softLimit.reverseSoftLimit(-0.6);
        wristPID = new ClosedLoopConfig();
        rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kBrake);
        rollerPID = new ClosedLoopConfig();

        wristPID.pidf(0,0,0,0);
        wristPID.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        rollerPID.pidf(0,0,0,0);

        wristConfig.apply(wristPID);
        rollerConfig.apply(rollerPID);

        applyConfigs();
        // --------------------------------------------------------------

    }

    private void applyConfigs() {
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void updateInputs(ManipulatorIOIn inputs) {
        inputs.beamBreakFwd = beamBreakFront.get();
        inputs.beamBreakBack = beamBreakBack.get();

        inputs.wristMotorRawPosition = wristMotor.getEncoder().getPosition();
        inputs.wristMotorOffsetPosition = inputs.wristMotorRawPosition - wristOffset;
        inputs.wristCalculatedPosition = absoluteEncoder.getPosition();
        inputs.wristMotorAngularVelocity = wristMotor.getEncoder().getVelocity();
        inputs.wristMotorDutyCycle = wristMotor.getAppliedOutput();
        inputs.wristMotorCurrent = wristMotor.getOutputCurrent();
        inputs.wristMotorBusVoltage = wristMotor.getBusVoltage();

        inputs.rollerMotorAngularVelocity = rollerMotor.getEncoder().getVelocity();
        inputs.rollerAngularVelocity = inputs.rollerMotorAngularVelocity * elevatorConstants.rollerDriveRatio;
        inputs.rollerSurfaceVelocity = inputs.rollerAngularVelocity * Math.PI * elevatorConstants.rollerDiameter;
        inputs.rollerMotorDutyCycle = rollerMotor.getAppliedOutput();
        inputs.rollerMotorCurrent = rollerMotor.getOutputCurrent();
        inputs.rollerMotorBusVoltage = rollerMotor.getBusVoltage();
    }

    public void seedWristPos(double currentRealPosition) {
        wristOffset = wristMotor.getAlternateEncoder().getPosition() - currentRealPosition;
    }

    public void setWristPos(double position, double FF) {
        wristMotor.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, FF);
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void updateWristPIDs(double P, double I, double D, double FF) {
        wristPID.pidf(P, I, D, FF);
        wristConfig.apply(wristPID);
        applyConfigs();
    }
}
