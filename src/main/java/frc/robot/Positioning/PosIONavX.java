package frc.robot.Positioning;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;

public class PosIONavX implements PosIO {

  AHRS navx;
 

  DoubleSubscriber estXSub;
  DoubleSubscriber estYSub;
  DoubleSubscriber estZSub;
  DoubleSubscriber kGainSub;

  DoublePublisher xAccelPub;
  DoublePublisher yAccelPub;
  DoublePublisher zAccelPub;

  public PosIONavX(AHRS navx) {
    // this.navx = navx;
    this.navx = new AHRS(NavXComType.kMXP_SPI);
  }

  @Override
  public void updateInputs(PosIOIn inputs) {
    inputs.xAngle = Rotation2d.fromDegrees(navx.getRoll());
    inputs.xAngleDegrees = inputs.xAngle.getDegrees();
    inputs.xAngularVelocity = Rotation2d.fromDegrees(navx.getRawGyroX());
    inputs.yAngle = Rotation2d.fromDegrees(navx.getPitch());
    inputs.yAngleDegrees = inputs.yAngle.getDegrees();
    inputs.yAngularVelocity = Rotation2d.fromDegrees(navx.getRawGyroY());
    inputs.zAngle = Rotation2d.fromDegrees(-navx.getYaw());
    inputs.zAngleDegrees = inputs.zAngle.getDegrees();
    inputs.zAngularVelocity = Rotation2d.fromDegrees(-navx.getRawGyroZ());

    inputs.xAccel = navx.getRawAccelX();
    inputs.yAccel = navx.getRawAccelY();
    inputs.zAccel = navx.getRawAccelZ();

  }

  public void zero() {
    navx.reset();
  }
}
