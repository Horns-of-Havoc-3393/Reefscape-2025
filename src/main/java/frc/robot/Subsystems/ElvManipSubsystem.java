package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.SparkMax;

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
    LoggedNetworkNumber wristSetpoint = new LoggedNetworkNumber("SmartDashboard/Elevator/wristSetpoint");

    double wristTarget = 0;
    double elvTarget = 0;
    double elvAdjustment = 0;

    int intakeDelay = 0;


    public enum setpoints {
        L1, L2, L3, L4, CORAL, STOW, DISLODGEL, DISLODGEH// for algae flicking
    }

    private class beamBreakSupplier implements BooleanSupplier {
        public boolean getAsBoolean() {
            System.out.println(manipInputs.beamBreakBack);
            return !manipInputs.beamBreakBack;
            // return false;
        }
    }
    
    public beamBreakSupplier hasCoral;

    int PIDUpdates = 0;

    public ElvManipSubsystem(SparkMax elevator1, SparkMax elevator2, SparkMax wristMotor, SparkMax rollerMotor) {
        elvIO = new ElevatorIOVortex(elevator1, elevator2);
        elvInputs = new ElevatorIOInAutoLogged();

        manipIO = new ManipulatorIONEO(wristMotor, rollerMotor);
        manipInputs = new ManipulatorIOInAutoLogged();

        elvIO.updateInputs(elvInputs);
        manipIO.updateInputs(manipInputs);
        Logger.processInputs("Elevator", elvInputs);
        Logger.processInputs("Manipulator", manipInputs);

        elvIO.setCurrentPosition(0.0);
        manipIO.seedWristPos(0.0);


        run.set(false);
        setpoint.set(0);
        wristSetpoint.set(0);
        updatePIDs.set(false);

        elvIO.updatePIDs(elevatorConstants.kelvP, elevatorConstants.kelvI, elevatorConstants.kelvD, 0, elevatorConstants.kelvVel, elevatorConstants.kelvAccel, elevatorConstants.kelvJerk);
        manipIO.updateWristPIDs(elevatorConstants.kmanipP, elevatorConstants.kmanipI, elevatorConstants.kmanipD, elevatorConstants.kmanipFF);

        hasCoral = new beamBreakSupplier();
    }
    
    public void periodic() {
        elvIO.updateInputs(elvInputs);
        manipIO.updateInputs(manipInputs);
        Logger.processInputs("Elevator", elvInputs);
        Logger.processInputs("Manipulator", manipInputs);

        if(manipInputs.wristCalculatedPosition < -0.23 || elvTarget < 4) {
            elvIO.setPosition(elvTarget + elvAdjustment);
        }
        if(elvInputs.motor1OffsetPosition < 4 || wristTarget < -0.23) {
            manipIO.setWristPos(wristTarget,0);
        }

        if(PIDUpdates<10) {

        }

        if(updatePIDs.get() || PIDUpdates<10) {
            elvIO.updatePIDs(elevatorConstants.elvP.get(), elevatorConstants.elvI.get(), elevatorConstants.elvD.get(), 0, elevatorConstants.elvVel.get(), elevatorConstants.elvAccel.get(), elevatorConstants.elvJerk.get());
            manipIO.updateWristPIDs(elevatorConstants.manipP.get(), elevatorConstants.manipI.get(), elevatorConstants.manipD.get(), elevatorConstants.manipFF.get());
            PIDUpdates++;
        }
        if(run.get()) {
            System.out.println("update PIDs");
            elvIO.setPosition(setpoint.get());
            manipIO.setWristPos(wristSetpoint.get(),0.0);
        }
    }

    // I know this function is basically useless rn but it will be expanded if we ever improve the manipulator code
    private void setState(double height, double wristPos) {
        elvTarget = height;
        wristTarget = wristPos;
        // elvIO.setPosition(height);
        // manipIO.setWristPos(wristPos, 0);
    }

    public void adjustHeight(double adjustment) {
        elvAdjustment = adjustment;
    }

    public void gotoSetpoint(setpoints target) {
        switch(target) {
            case L2:
                setState(0, -0.2);
                break;
            case L3:
                setState(14.3,-0.25);
                break;
            case L4:
                setState(34.9,-0.25);
                break;
            case L1:
                setState(42,-0.25);
                break;
            case CORAL:
                setState(4.238,0.0);
                normal_in();
                break;
            case STOW:
                setState(0.0,0.0);
                stopRollers();
                break;
            case DISLODGEL:
                setState(4.0,-20);
                normal_out();
                break;
            case DISLODGEH:
                setState(17.7, -20);
                normal_out();
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
        System.out.println(manipInputs.beamBreakBack);
        if(manipInputs.beamBreakBack && manipInputs.beamBreakFwd) {
            manipIO.setRollerSpeed(-1);
            intakeDelay = 0;
        }else if(manipInputs.beamBreakBack && !manipInputs.beamBreakFwd){
            if(intakeDelay>7) {
                manipIO.setRollerSpeed(-0.5);
            }
            intakeDelay++;
        } else {
            manipIO.setRollerSpeed(0);
            intakeDelay = 0;
        }
    }
    public void stopRollers(){
        manipIO.setRollerSpeed(0);
    }
}
