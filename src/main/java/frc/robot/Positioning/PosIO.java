package frc.robot.Positioning;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PosIO {
  @AutoLog
  public static class PosIOIn {
    public Rotation2d xAngle = new Rotation2d(0.0);
    public double xAngleDegrees = 0.0;
    public Rotation2d xAngularVelocity = new Rotation2d(0.0);

    public Rotation2d yAngle = new Rotation2d(0.0);
    public double yAngleDegrees = 0.0;
    public Rotation2d yAngularVelocity = new Rotation2d(0.0);

    public Rotation2d zAngle = new Rotation2d(0.0);
    public double zAngleDegrees = 0.0;
    public Rotation2d zAngularVelocity = new Rotation2d(0.0);

    public double xAccel = 0.0;
    public double yAccel = 0.0;
    public double zAccel = 0.0;

  }

  public default void updateInputs(PosIOIn inputs) {}
}
