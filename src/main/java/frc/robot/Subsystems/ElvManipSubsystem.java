package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

public class ElvManipSubsystem extends SubsystemBase{

    ElevatorIOVortex elvIO;
    ElevatorIOInAutoLogged elvInputs;

    ManipulatorIONEO manipIO;
    ManipulatorIOInAutoLogged manipInputs;

    LoggedNetworkBoolean run = new LoggedNetworkBoolean("SmartDashboard/Elevator/run");
    LoggedNetworkBoolean updatePIDs = new LoggedNetworkBoolean("SmartDashboard/Elevator/updatePIDs");
    
    LoggedNetworkNumber setpoint = new LoggedNetworkNumber("SmartDashboard/Elevator/setpoint");

    public enum setpoints {
        L1, L2, L3, L4, CORAL, STOW, DISLODGE// for algae flicking
    }

    public ElvManipSubsystem(SparkMax elevator1, SparkMax elevator2, SparkMax wristMotor, SparkMax rollerMotor) {
        elvIO = new ElevatorIOVortex(elevator1, elevator2);
        elvInputs = new ElevatorIOInAutoLogged();

        manipIO = new ManipulatorIONEO(wristMotor, rollerMotor);
        manipInputs = new ManipulatorIOInAutoLogged();

        elvIO.updateInputs(elvInputs);
        manipIO.updateInputs(manipInputs);
        Logger.processInputs("Elevator", elvInputs);
        Logger.processInputs("Manipulator", manipInputs);


        run.set(false);
        setpoint.set(0);
        updatePIDs.set(false);
    }
    
    public void periodic() {
        elvIO.updateInputs(elvInputs);
        Logger.processInputs("Elevator", elvInputs);

        if(updatePIDs.get()) {
            elvIO.updatePIDs(elevatorConstants.elvP.get(), elevatorConstants.elvI.get(), elevatorConstants.elvD.get(), 0, elevatorConstants.elvVel.get(), elevatorConstants.elvAccel.get(), elevatorConstants.elvJerk.get());
            manipIO.updateWristPIDs(elevatorConstants.manipP.get(), elevatorConstants.manipI.get(), elevatorConstants.manipD.get(), elevatorConstants.manipFF.get());
        }
        // if(run.get()) {
        //     elvIO.setPosition(setpoint.get());
        // }
    }

    // I know this function is basically useless rn but it will be expanded if we ever improve the manipulator code
    private void setState(double height, double wristPos) {
        elvIO.setPosition(height);
        manipIO.setWristPos(wristPos, 0);
    }

    public void gotoSetpoint(setpoints target) {
        switch(target) {
            case L1:
                setState(0.0, 0.0);
                break;
            case L2:
                setState(0.0,0.0);
                break;
            case L3:
                setState(0.0,0.0);
                break;
            case L4:
                setState(0.0,0.0);
                break;
            case CORAL:
                setState(0.0,0.0);
                break;
            case STOW:
                setState(0.0,0.0);
                break;
            case DISLODGE:
                setState(0.0,0.0);
                break;
            
        }
    }
    public void setRollPercentage(double percent){
        manipIO.setRollerSpeed(percent);
    }
    public void Slow_in(){
        manipIO.setRollerSpeed(-0.5);
    }
    public void normal_out(){
        manipIO.setRollerSpeed(0.75);
    }
    public void normal_in(){
        manipIO.setRollerSpeed(-0.75);
    }
}
