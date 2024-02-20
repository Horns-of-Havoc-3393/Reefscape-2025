package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModIO {
  @AutoLog
  public static class ModIOIn {
    double driveVelocityRPS;
    double driveVelocityMPS;
    double steerVelocityRPS;
    double driveCurrentAmps;
    double steerCurrentAmps;
    double driveVolts;
    double steerVolts;
    Rotation2d steerPosRelative;
    Rotation2d steerPosAbsolute;
    double steerPosRaw;
  }

  public default void updateInputs(ModIOIn inputs) {}
}
