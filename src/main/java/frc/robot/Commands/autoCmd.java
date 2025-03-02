package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.SwerveBase;

public class autoCmd extends Command {
  SwerveBase swerve;

  long startTime;

  public autoCmd(SwerveBase swerve) {
    this.swerve = swerve;
    this.addRequirements(this.swerve);


    swerve.updatePIDs();

    System.out.println("init auto");
  }

  @Override
  public void initialize() {
    startTime = RobotController.getFPGATime();
  }

  @Override
  public void execute() {
    swerve.setFO(new ChassisSpeeds(2, 0, 0), 5);
  }

  @Override
  public boolean isFinished() {
    return RobotController.getFPGATime() > startTime + 1000000;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setFO(new ChassisSpeeds(0,0,0), 5);
  }
}
