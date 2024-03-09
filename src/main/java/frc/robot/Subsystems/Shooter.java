package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  ShooterIOSparks io;
  ShooterIOInAutoLogged inputs;

  public static class angleSetpoints {
    double SHOOT = 60;
    double LOAD = 50;
    double AMP = 110;
    double DRIVE = 0;
  }

  public Shooter(
      CANSparkFlex topMotor,
      CANSparkFlex bottomMotor,
      CANSparkFlex conveyorMotor,
      CANSparkMax leftElevator,
      CANSparkMax rightElevator) {


    io = new ShooterIOSparks(topMotor,bottomMotor,conveyorMotor,leftElevator,rightElevator);
    inputs = new ShooterIOInAutoLogged();
  }
  
  public void setpoints() {

  }
}
