package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldCentricWithProperDeadband extends FieldCentric {
  public double MaxSpeed = 0;

  public double MaxAngularSpeed = 0;

  public boolean CreepEnable;

  public double CreepProp = 0.25;

  public double CreepRotProp = 0.25;

  private Rotation2d[] lastRotations = new Rotation2d[4];

  @Override
  public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
    double x_direction = VelocityX / Math.abs(VelocityX);
    double y_direction = VelocityY / Math.abs(VelocityY);
    double rot_direction = RotationalRate / Math.abs(RotationalRate);

    double toApplyX = x_direction * Math.max(Math.abs(VelocityX) - Deadband, 0) / (MaxSpeed - Deadband) * MaxSpeed;
    double toApplyY = y_direction * Math.max(Math.abs(VelocityY) - Deadband, 0) / (MaxSpeed - Deadband) * MaxSpeed;
    double toApplyOmega = rot_direction * Math.max(Math.abs(RotationalRate) - RotationalDeadband, 0)
        / (MaxAngularSpeed - RotationalDeadband) * MaxAngularSpeed;

    toApplyX *= CreepEnable ? CreepProp : 1;
    toApplyY *= CreepEnable ? CreepProp : 1;
    toApplyOmega *= CreepEnable ? CreepRotProp : 1;

    ChassisSpeeds speeds = ChassisSpeeds
        .discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
            parameters.currentPose.getRotation()), parameters.updatePeriod);

    var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

    var startTime = Timer.getFPGATimestamp();
    for (int i = 0; i < modulesToApply.length; ++i) {
      if (Math.abs(states[i].speedMetersPerSecond) < 0.01) {
        if (lastRotations[i] == null) {
          lastRotations[i] = new Rotation2d();
        }
        states[i].angle = lastRotations[i];
      }

      modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      lastRotations[i] = states[i].angle;
    }
    SmartDashboard.putNumber("Request Loop Time Ms", (Timer.getFPGATimestamp() - startTime) * 1000.0);

    return StatusCode.OK;
  }

  public FieldCentricWithProperDeadband withMaxSpeed(double maxSpeed) {
    this.MaxSpeed = maxSpeed;
    return this;
  }

  public FieldCentricWithProperDeadband withMaxAngularSpeed(double maxAngularSpeed) {
    this.MaxAngularSpeed = maxAngularSpeed;
    return this;
  }

  public FieldCentricWithProperDeadband withCreepEnabled(boolean creep) {
    this.CreepEnable = creep;
    return this;
  }

  /**
   * Sets the velocity in the X direction, in m/s.
   * X is defined as forward according to WPILib convention,
   * so this determines how fast to travel forward.
   *
   * @param velocityX Velocity in the X direction, in m/s
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withVelocityX(double velocityX) {
    this.VelocityX = velocityX;
    return this;
  }

  /**
   * Sets the velocity in the Y direction, in m/s.
   * Y is defined as to the left according to WPILib convention,
   * so this determines how fast to travel to the left.
   *
   * @param velocityY Velocity in the Y direction, in m/s
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withVelocityY(double velocityY) {
    this.VelocityY = velocityY;
    return this;
  }

  /**
   * The angular rate to rotate at, in radians per second.
   * Angular rate is defined as counterclockwise positive,
   * so this determines how fast to turn counterclockwise.
   *
   * @param rotationalRate Angular rate to rotate at, in radians per second
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withRotationalRate(double rotationalRate) {
    this.RotationalRate = rotationalRate;
    return this;
  }

  /**
   * Sets the allowable deadband of the request.
   *
   * @param deadband Allowable deadband of the request
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withDeadband(double deadband) {
    this.Deadband = deadband;
    return this;
  }

  /**
   * Sets the rotational deadband of the request.
   *
   * @param rotationalDeadband Rotational deadband of the request
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withRotationalDeadband(double rotationalDeadband) {
    this.RotationalDeadband = rotationalDeadband;
    return this;
  }

  /**
   * Sets the type of control request to use for the drive motor.
   *
   * @param driveRequestType The type of control request to use for the drive
   *                         motor
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
    this.DriveRequestType = driveRequestType;
    return this;
  }

  /**
   * Sets the type of control request to use for the steer motor.
   *
   * @param steerRequestType The type of control request to use for the steer
   *                         motor
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
    this.SteerRequestType = steerRequestType;
    return this;
  }
}