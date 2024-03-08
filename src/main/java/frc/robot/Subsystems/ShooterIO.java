package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.shooterConstants;

public class ShooterIO implements ShooterIOBase {
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

    public ShooterIO(CANSparkFlex shooter1, CANSparkFlex shooter2, CANSparkFlex conveyor, CANSparkMax elevator1, CANSparkMax elevator2) {
        this.shooter1 = shooter1;
        this.shooter2 = shooter2;
        this.conveyor = conveyor; 
    }

    public void updateInputs(ShooterIOIn inputs) {
        inputs.shooter1DutyCycle = shooter1.getAppliedOutput();
        inputs.shooter1Volts = shooter1.getBusVoltage();
        inputs.shooter1SpeedRPS = shooter1.getAbsoluteEncoder().getVelocity();
        inputs.shooter1SpeedMPS = inputs.shooter1SpeedRPS*(shooterConstants.shootWheelDiameter*Math.PI);
        inputs.shooter1SetpointMPS = shooter1SetpointMPS;
        inputs.shooter1SetpointRPS = shooter1SetpointRPS;

        
        inputs.shooter2DutyCycle = shooter2.getAppliedOutput();
        inputs.shooter2Volts = shooter2.getBusVoltage();
        inputs.shooter2SpeedRPS = shooter2.getAbsoluteEncoder().getVelocity();
        inputs.shooter2SpeedMPS = inputs.shooter2SpeedRPS*(shooterConstants.shootWheelDiameter*Math.PI);
        inputs.shooter2SetpointMPS = shooter2SetpointMPS;
        inputs.shooter2SetpointRPS = shooter2SetpointRPS;

        inputs.elevator1DutyCycle = elevator1.getAppliedOutput();
        inputs.elevator1Volts = elevator1.getBusVoltage();
        inputs.elevator1SpeedRPS = elevator1.getAbsoluteEncoder().getVelocity();
        inputs.elevator1SetpointDeg = elevator1SetpointDeg;
        inputs.elevator1Position = Rotation2d.fromRotations(elevator1.getAbsoluteEncoder().getPosition());

        inputs.elevator2DutyCycle = elevator1.getAppliedOutput();
        inputs.elevator2Volts = elevator1.getBusVoltage();
        inputs.elevator2SpeedRPS = elevator1.getAbsoluteEncoder().getVelocity();
        inputs.elevator2SetpointDeg = elevator2SetpointDeg;
        inputs.elevator2Position = Rotation2d.fromRotations(elevator2.getAbsoluteEncoder().getPosition());

        inputs.shooter2DutyCycle = shooter2.getAppliedOutput();
        inputs.shooter2Volts = shooter2.getBusVoltage();
        inputs.shooter2SpeedRPS = shooter2.getAbsoluteEncoder().getVelocity();
        inputs.shooter2SpeedMPS = inputs.shooter2SpeedRPS*(shooterConstants.shootWheelDiameter*Math.PI);
        inputs.shooter2SetpointMPS = shooter2SetpointMPS;
        inputs.shooter2SetpointRPS = shooter2SetpointRPS;

        inputs.conveyorDutyCycle = conveyor.getAppliedOutput();
        inputs.conveyorVolts = conveyor.getBusVoltage();
        inputs.conveyorSpeedRPS = conveyor.getEncoder().getPosition();

        inputs.shooterAngleAbs = Rotation2d.fromRotations(absEncoder.getAbsolutePosition());

        inputs.beamBreak = beamBreak.get();
    }

    public void setShooterSpeed(double speed1MPS, double speed2MPS) {
        shooter1SetpointMPS = speed1MPS;
        shooter2SetpointMPS = speed2MPS;
        shooter1SetpointRPS = speed1MPS/(shooterConstants.shootWheelDiameter*Math.PI);
        shooter2SetpointRPS = speed2MPS/(shooterConstants.shootWheelDiameter*Math.PI);
        shooter1.getPIDController().setReference(shooter1SetpointRPS,CANSparkBase.ControlType.kVelocity);
        shooter2.getPIDController().setReference(shooter2SetpointRPS,CANSparkBase.ControlType.kVelocity);
    }

    public void setElevatorAngle(Rotation2d angle) {
        double motorPos = angle.getRotations()*shooterConstants.elevatorConversion;
        elevator1.getPIDController().setReference(motorPos, CANSparkBase.ControlType.kPosition);
        elevator2.getPIDController().setReference(motorPos, CANSparkBase.ControlType.kPosition);
    }

    public void setInversions(Boolean shooter1, Boolean shooter2, Boolean conveyor) {
        this.shooter1.setInverted(shooter1);
        this.shooter2.setInverted(shooter2);
        this.conveyor.setInverted(conveyor);
    }
}
