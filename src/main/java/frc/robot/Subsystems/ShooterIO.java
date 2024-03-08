package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.shooterConstants;

public class ShooterIO implements ShooterIOBase {
    CANSparkFlex shooter1;
    CANSparkFlex shooter2;
    CANSparkFlex conveyor;

    DigitalInput beamBreak;

    double shooter1SetpointMPS;
    double shooter1SetpointRPS;

    double shooter2SetpointMPS;
    double shooter2SetpointRPS;

    public ShooterIO(CANSparkFlex shooter1, CANSparkFlex shooter2, CANSparkFlex conveyor) {
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

        inputs.conveyorDutyCycle = conveyor.getAppliedOutput();
        inputs.conveyorVolts = conveyor.getBusVoltage();
        inputs.conveyorSpeedRPS = conveyor.getEncoder().getPosition();

        inputs.beamBreak = beamBreak.get();
    }
}
