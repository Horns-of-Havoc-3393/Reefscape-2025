package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.driveConstants;
import frc.robot.Subsystems.Drive.SwerveBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class SwerveAbs extends Command {

  SwerveBase swerve;

  CommandXboxController controller;

  LoggedNetworkNumber deadzone;
  LoggedNetworkNumber lateralMaxSpeed;
  LoggedNetworkNumber rotationalMaxSpeed;

  SlewRateLimiter xLimit;
  SlewRateLimiter yLimit;
  SlewRateLimiter rLimit;

  LoggedNetworkNumber latAccLimit;
  LoggedNetworkNumber rotAccLimit;
  LoggedNetworkBoolean update;

  public SwerveAbs(SwerveBase swerve, CommandXboxController controller) {
    this.controller = controller;
    this.swerve = swerve;

    this.addRequirements(swerve);
    System.out.println("initCMD");

    deadzone = new LoggedNetworkNumber("/SmartDashboard/Control/deadzone");
    lateralMaxSpeed = new LoggedNetworkNumber("/SmartDashboard/Control/lateralMaxSpeed");
    lateralMaxSpeed.set(driveConstants.maxSpeedMPS);
    rotationalMaxSpeed = new LoggedNetworkNumber("/SmartDashboard/Control/rotationalMaxSpeed");
    latAccLimit = new LoggedNetworkNumber("/SmartDashboard/Control/LateralAcceleration");
    latAccLimit.set(driveConstants.lateralAccelLimitMPSPS);
    rotAccLimit = new LoggedNetworkNumber("/SmartDashboard/Control/RotationalAcceleration");
    rotAccLimit.set(driveConstants.rotationalAccelLimitRPSPS);
    update = new LoggedNetworkBoolean("/SmartDashboard/update");

    xLimit = new SlewRateLimiter(driveConstants.lateralAccelLimitMPSPS);
    yLimit = new SlewRateLimiter(driveConstants.lateralAccelLimitMPSPS);
    rLimit = new SlewRateLimiter(driveConstants.rotationalAccelLimitRPSPS);
  }

  public void execute() {
    double initial = RobotController.getFPGATime();
    Logger.recordOutput("Drive/AbsCMD/xAxis", controller.getLeftY());
    Logger.recordOutput("Drive/AbsCMD/yAxis", controller.getLeftX());
    Logger.recordOutput("Drive/AbsCMD/betaAxis", controller.getRightX());
    if (update.get()) {
      xLimit = new SlewRateLimiter(latAccLimit.get());
      yLimit = new SlewRateLimiter(latAccLimit.get());
      rLimit = new SlewRateLimiter(rotAccLimit.get());
    }
    double maxSpeed = lateralMaxSpeed.get();
    if ((Math.pow(controller.getLeftY(), 2)
            + Math.pow(controller.getLeftX(), 2)
            + Math.pow(controller.getRightX(), 2))
        > driveConstants.deadZone) {
      swerve.setFO(
          // new ChassisSpeeds(
          //     xLimit.calculate(controller.getLeftY() * -1 * maxSpeed),
          //     yLimit.calculate(controller.getLeftX() * -1 * maxSpeed),
          //     rLimit.calculate(controller.getRightX() * -1 * driveConstants.maxRotRPS))
          new ChassisSpeeds(
              xLimit.calculate(controller.getLeftY() *-1* maxSpeed),
              yLimit.calculate(controller.getLeftX()  *-1* maxSpeed),
              rLimit.calculate(controller.getRightX() *1* driveConstants.maxRotRPS)),
          5);
          //new comment
    } else {
      swerve.setFO(
          new ChassisSpeeds(xLimit.calculate(0), yLimit.calculate(0), rLimit.calculate(0)),
          driveConstants.maxSpeedMPS);
    }
    Logger.recordOutput("Timers/SwerveAbsEx", (RobotController.getFPGATime() - initial) * 0.000001);
  }
}
