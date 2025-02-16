package frc.robot.LimeLight;

import edu.wpi.first.networktables.NetworkTable;

public class LimeLightInterface {
    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public double getDoubleEntry(String entry){
        return limelight.getEntry(entry).getDouble(0);
    }
    public double[] getDoubleArrayEntry(String entry){
        return limelight.getEntry(entry).getDoubleArray(new double[0]);
    }
    public double getATagID(){
        return getDoubleEntry("tid");
    }
    public double getTargetArea(){
        return getDoubleEntry("ta");
    }
    public boolean hasValidTargets(){
        return getDoubleEntry("tv") == 1.0;
    }
    public double getXOffset(){
        return getDoubleEntry("tx");
    }
    public double[] getBotPose(){
        return getDoubleArrayEntry("botpose_targetspace");
    }
}
