package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;

public class ElevatorIOVortex implements ElevatorIO {
    SparkFlex motor1;
    SparkFlex motor2;

    RelativeEncoder encoder1;
    RelativeEncoder encoder2;

    SparkClosedLoopController pid1;
    SparkClosedLoopController pid2;

    double encoderOffset1;
    double encoderOffset2;
    
    public ElevatorIOVortex(SparkFlex motor1, SparkFlex motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;

        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();

        pid1 = motor1.getClosedLoopController();
        pid2 = motor2.getClosedLoopController();


        // Configure Motors ---------------------------------------------------------------
        ClosedLoopConfig config = new ClosedLoopConfig();
        config.pid(0,0,0);

        // --------------------------------------------------------------------------------

        // Set encoder offsets
        encoderOffset1 = encoder1.getPosition();
        encoderOffset2 = encoder2.getPosition();
    }

    public void updateInputs(ElevatorIOIn inputs) {
        inputs.motor1RawPosition = encoder1.getPosition();
        inputs.motor1OffsetPosition = inputs.motor1RawPosition - encoderOffset1;

        inputs.motor1Velocity = encoder1.getVelocity();

        inputs.motor1DutyCycle = motor1.getAppliedOutput();
        inputs.motor1BusVoltage = motor1.getBusVoltage();
        inputs.motor1Current = motor1.getOutputCurrent();
        inputs.motor1Inverted = motor1.configAccessor.getInverted();




        inputs.motor2RawPosition = encoder2.getPosition();
        inputs.motor2OffsetPosition = inputs.motor2RawPosition - encoderOffset2;

        inputs.motor2Velocity = encoder2.getVelocity();

        inputs.motor2DutyCycle = motor2.getAppliedOutput();
        inputs.motor2BusVoltage = motor2.getBusVoltage();
        inputs.motor2Current = motor2.getOutputCurrent();
        inputs.motor2Inverted = motor2.configAccessor.getInverted();
    }

    // sets the position of both motors *in motor rotations* (0 is the bottom position)
    public void setPosition(double position) {
        pid1.setReference(position+encoderOffset1, ControlType.kPosition);
        pid2.setReference(position+encoderOffset2, ControlType.kPosition);
    }

}
