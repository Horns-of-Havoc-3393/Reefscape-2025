package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.SwerveBase;
import java.util.function.DoubleSupplier;

public class SwerveAbs extends Command {

  SwerveBase swerve;

  DoubleSupplier xAxis;
  DoubleSupplier yAxis;
  DoubleSupplier betaAxis;

  public SwerveAbs(
      SwerveBase swerve, DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier betaAxis) {
    this.swerve = swerve;

    this.addRequirements(swerve);
  }

  @Override
  public void execute() {
    if ((Math.pow(xAxis.getAsDouble(), 2) + Math.pow(yAxis.getAsDouble(), 2)) > 0.1) {
      swerve.setFO(
          new ChassisSpeeds(xAxis.getAsDouble(), yAxis.getAsDouble(), betaAxis.getAsDouble()));
    }
  }
}
