package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants.autonConstants;
import frc.robot.Positioning.PosIOInAutoLogged;
import frc.robot.Positioning.PosIONavX;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class SwerveBase extends SubsystemBase {
  PosIONavX posIO;
  PosIOInAutoLogged inputs;

  SwerveMod[] modules = new SwerveMod[4];

  SwerveDriveKinematics kinematics;

  DoublePublisher xVelPub;
  DoublePublisher yVelPub;

  LoggedNetworkBoolean update;
  LoggedNetworkBoolean zeroGyro;
  LoggedNetworkBoolean publishTargetStates;

  SwerveDrivePoseEstimator estimator;

  int PIDUpdates = 0;

  private double initialTimestamp;

  public SwerveBase(
      TalonFX[] driveMotors,
      TalonFX[] steerMotors,
      CANcoder[] encoders,
      Translation2d[] positions,
      Rotation2d[] absEncoderOffsets,
      PosIONavX posIO)
    {
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(positions);
    this.kinematics = kinematics;

    // instantiate swerve mod classes
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveMod(driveMotors[i], steerMotors[i], encoders[i], absEncoderOffsets[i]);
    }



    this.posIO = posIO;
    inputs = new PosIOInAutoLogged();


    // debug telemetry stuff
    update = new LoggedNetworkBoolean("/SmartDashboard/update", false);
    zeroGyro = new LoggedNetworkBoolean("/SmartDashboard/Control/zeroGyro", false);
    publishTargetStates = new LoggedNetworkBoolean("/SmartDashboard/Control/publishTargetStates", false);



    posIO.updateInputs(inputs);
    Logger.processInputs("Positioning", inputs);


    // Zero gyro and set PIDs of each module
    posIO.zero();
    for (int i = 0; i < 4; i++) {
      modules[i].updatePIDs();
    }


    // init estimator
    estimator = new SwerveDrivePoseEstimator(
      this.kinematics,
      inputs.zAngle,
      getPositions(),
      new Pose2d(0,0,inputs.zAngle),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    // AutoBuilder init
    AutoBuilder.configure(
      this::getPose,
      this::setPose,
      this::getChassisSpeeds,
      (speeds, feedforwards) -> setRelativeSpeeds(speeds), 
      new PPHolonomicDriveController(
        new PIDConstants(autonConstants.ktransP,autonConstants.ktransI,autonConstants.ktransD),
        new PIDConstants(autonConstants.krotP,autonConstants.krotI,autonConstants.krotD)
      ),
      autonConstants.pathPlannerConfig,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this);
  }


  // Moves the robot according to the field-oriented "speeds" ChassisSpeeds object
  public void setFieldOrientedSpeeds(ChassisSpeeds speeds, double lateralMaxSpeed) {
    double initial = RobotController.getFPGATime();

    SwerveModuleState[] states =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, inputs.zAngle));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, lateralMaxSpeed);

    Logger.recordOutput("Drive/targetStates", states);

    for (int i = 0; i < 4; i++) {
      modules[i].setSwerveState(states[i]);
    }
    Logger.recordOutput("Timers/SwerveBaseSetFO", (RobotController.getFPGATime() - initial) * 0.000001);
  }


  // Set drive speeds relative to the robot
  public void setRelativeSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(180)));

    Logger.recordOutput("Drive/relativeTargetSpeeds", speeds);
    Logger.recordOutput("Drive/targetStates", states);

    for (int i=0; i<4; i++) {
      modules[i].setSwerveState(states[i]);
    }
  }


  // Get the current state of each swerve module
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }


  // Get the current position of each swerve module (uses distance traveled by drive motor rather than current velocity)
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }


  // Get the current target states for each module (for logging/debug purposes)
  public SwerveModuleState[] getTargetStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getTargetState();
    }
    return states;
  }


  // returns current chassis speeds of the robot
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }



  // returns current estimator pose of the robot
  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }



  // set the current estimator pose
  public void setPose(Pose2d newPose) {
    estimator.resetPose(newPose);
  }


  // zero the gryo
  public void zeroGyro() {
    posIO.zero();
  }

  public void updatePIDs() {
    for (int i=0; i<4; i++) {
      modules[i].updatePIDs();
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Timers/SwerveBasePdFreq", 1 / ((RobotController.getFPGATime() - initialTimestamp) * 0.000001));
    initialTimestamp = RobotController.getFPGATime();



    posIO.updateInputs(inputs);
    Logger.processInputs("Positioning", inputs);


    // debug stuff
    if (zeroGyro.get()) {
      posIO.zero();
    }

    for (var module : modules) {
      module.periodic();
      if (update.get() || PIDUpdates < 10) {
        System.out.println("Update base pids");
        module.updatePIDs();
        PIDUpdates++;
      }
    }



    // logging/debug
    SwerveModuleState[] states = getStates();
    SwerveModuleState[] targetStates = getTargetStates();
    Logger.recordOutput("Drive/swerveState", states);
    
    if(targetStates[0] != null){
      Logger.recordOutput("Drive/realTargetStates", targetStates);
    }


    // step estimator
    estimator.update(inputs.zAngle, getPositions());
    LimelightHelpers.SetRobotOrientation("limelight", estimator.getEstimatedPosition().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    PoseEstimate visionEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (visionEst != null) {
      estimator.addVisionMeasurement(visionEst.pose, visionEst.timestampSeconds);
    }


    Logger.recordOutput("ChassisAngle", inputs.zAngle);
    Logger.recordOutput(
        "Timers/SwerveBasePd", (RobotController.getFPGATime() - initialTimestamp) * 0.000001);
  }
}
