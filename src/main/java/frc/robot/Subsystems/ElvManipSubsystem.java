package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

public class ElvManipSubsystem extends SubsystemBase{
    ElevatorIOVortex elvIO;
    ElevatorIOInAutoLogged elvInputs;

    LoggedNetworkBoolean run = new LoggedNetworkBoolean("SmartDashboard/Elevator/run");
    LoggedNetworkBoolean updatePIDs = new LoggedNetworkBoolean("SmartDashboard/Elevator/updatePIDs");
    
    LoggedNetworkNumber setpoint = new LoggedNetworkNumber("SmartDashboard/Elevator/setpoint");

    public ElvManipSubsystem(SparkMax elevator1, SparkMax elevator2) {
        elvIO = new ElevatorIOVortex(elevator1, elevator2);
        elvInputs = new ElevatorIOInAutoLogged();

        elvIO.updateInputs(elvInputs);
        Logger.processInputs("Elevator", elvInputs);


        run.set(false);
        setpoint.set(0);
        updatePIDs.set(false);
    }
    
    public void periodic() {
        elvIO.updateInputs(elvInputs);
        Logger.processInputs("Elevator", elvInputs);

        if(updatePIDs.get()) {
            elvIO.updatePIDs(elevatorConstants.elvP.get(), elevatorConstants.elvI.get(), elevatorConstants.elvD.get(), 0, elevatorConstants.elvVel.get(), elevatorConstants.elvAccel.get(), elevatorConstants.elvJerk.get());
        }
        if(run.get()) {
            elvIO.setPosition(setpoint.get());
        }
    }
}
