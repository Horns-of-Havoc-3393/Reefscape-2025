package frc.robot.Subsystems;

public interface ShooterIOBase {
    public static class ShooterIOIn {
        double shooter1Volts;
        double shooter1DutyCycle;
        double shooter1SpeedRPS;
        double shooter1SpeedMPS;
        double shooter1SetpointRPS;
        double shooter1SetpointMPS;

        double shooter2Volts;
        double shooter2DutyCycle;
        double shooter2SpeedRPS;
        double shooter2SpeedMPS;
        double shooter2SetpointRPS;
        double shooter2SetpointMPS;

        double conveyorVolts;
        double conveyorDutyCycle;
        double conveyorSpeedRPS;

        Boolean beamBreak;

        double noteSpeed;
    }
}

