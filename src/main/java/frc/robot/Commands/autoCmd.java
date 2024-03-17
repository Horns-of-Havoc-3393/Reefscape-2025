package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.SwerveBase;

public class autoCmd extends Command {
  SwerveBase swerve;

  public autoCmd(SwerveBase swerve) {
    this.swerve = swerve;
    this.addRequirements(this.swerve);
  }

  public void execute() {
    swerve.setFO(new ChassisSpeeds(-2, 0, 0), 5);
  }
}
