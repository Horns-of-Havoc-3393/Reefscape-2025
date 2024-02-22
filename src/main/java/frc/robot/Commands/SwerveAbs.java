package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.SwerveBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class SwerveAbs extends Command {

  SwerveBase swerve;

  CommandXboxController controller;

  LoggedDashboardNumber deadzone;
  LoggedDashboardNumber lateralMaxSpeed;
  LoggedDashboardNumber rotationalMaxSpeed;

  public SwerveAbs(SwerveBase swerve, CommandXboxController controller) {
    this.controller = controller;
    this.swerve = swerve;

    this.addRequirements(swerve);
    System.out.println("initCMD");

    deadzone = new LoggedDashboardNumber("Control/deadzone");
    lateralMaxSpeed = new LoggedDashboardNumber("Control/lateralMaxSpeed");
    rotationalMaxSpeed = new LoggedDashboardNumber("Control/rotationalMaxSpeed");
  }

  public void execute() {
    double initial = Logger.getRealTimestamp();
    Logger.recordOutput("Drive/AbsCMD/xAxis", controller.getLeftY());
    Logger.recordOutput("Drive/AbsCMD/yAxis", controller.getLeftX());
    Logger.recordOutput("Drive/AbsCMD/betaAxis", controller.getRightX());
    if ((Math.pow(controller.getLeftY(), 2)
            + Math.pow(controller.getLeftX(), 2)
            + Math.pow(controller.getRightX(), 2))
        > deadzone.get()) {
      swerve.setFO(
          new ChassisSpeeds(
              controller.getLeftY() * lateralMaxSpeed.get(),
              controller.getLeftX() * lateralMaxSpeed.get(),
              controller.getRightX() * rotationalMaxSpeed.get()),
          lateralMaxSpeed.get());
    } else {
      swerve.setFO(new ChassisSpeeds(0, 0, 0), lateralMaxSpeed.get());
    }
    Logger.recordOutput("Timers/SwerveAbsEx", (Logger.getRealTimestamp() - initial) * 0.000001);
  }
}
