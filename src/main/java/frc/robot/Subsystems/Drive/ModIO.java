package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModIO {
  @AutoLog
  public static class ModIOIn {
    double driveVelocityRPS;
    double driveVelocityMPS;
    double driveCurrentAmps;
    double driveVolts;
    double driveVelErr;
    double driveDutyCycle;
    double drivePosition;
    double steerVelocityRPS;
    double steerCurrentAmps;
    double steerVolts;
    Rotation2d relEncoderCalculatedAngle;
    double relEncoderRaw;
    Rotation2d absEncoderCalculatedAngle;
    double steerPosDegrees;
    double steerPosErr;
    double currentLimit;
    Rotation2d targetAngle;
    double targetSpeed;
  }

  public default void updateInputs(ModIOIn inputs) {}
}
