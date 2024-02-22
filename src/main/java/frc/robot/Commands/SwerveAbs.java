package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.SwerveBase;
import org.littletonrobotics.junction.Logger;

public class SwerveAbs extends Command {

  SwerveBase swerve;

  CommandXboxController controller;

  public SwerveAbs(SwerveBase swerve, CommandXboxController controller) {
    this.controller = controller;
    this.swerve = swerve;

    this.addRequirements(swerve);
    System.out.println("initCMD");
  }

  public void execute() {
    Logger.recordOutput("Drive/AbsCMD/xAxis", controller.getLeftY());
    Logger.recordOutput("Drive/AbsCMD/yAxis", controller.getLeftX());
    Logger.recordOutput("Drive/AbsCmd/betaAxis", controller.getRightX());
    if ((Math.pow(controller.getLeftY(), 2)
            + Math.pow(controller.getLeftX(), 2)
            + Math.pow(controller.getRightX(), 2))
        > 0.01) {
      swerve.setFO(
          new ChassisSpeeds(
              controller.getLeftY() * -4.5,
              controller.getLeftX() * -4.5,
              controller.getRightX() * -4));
    } else {
      swerve.setFO(new ChassisSpeeds(0, 0, 0));
    }
  }
}
