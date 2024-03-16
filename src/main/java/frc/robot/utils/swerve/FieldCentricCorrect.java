package frc.robot.utils.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class FieldCentricCorrect extends FieldCentric {

    Rotation2d[] cachedModuleRotations = new Rotation2d[4];

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        double toApplyX = VelocityX;
        double toApplyY = VelocityY;
        if (ForwardReference == SwerveRequest.ForwardReference.OperatorPerspective) {
            /* If we're operator perspective, modify the X/Y translation by the angle */
            Translation2d tmp = new Translation2d(toApplyX, toApplyY);
            tmp = tmp.rotateBy(parameters.operatorForwardDirection);
            toApplyX = tmp.getX();
            toApplyY = tmp.getY();
        }
        double toApplyOmega = RotationalRate;
        if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
            toApplyX = 0;
            toApplyY = 0;
        }
        if (Math.abs(toApplyOmega) < RotationalDeadband) {
            toApplyOmega = 0;
        }

        ChassisSpeeds speeds = ChassisSpeeds
                .discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                        parameters.currentPose.getRotation()), parameters.updatePeriod);

        var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

        var keepSteerStill = true;

        for (SwerveModuleState state : states) {
            if (state.speedMetersPerSecond > 0.005) {
                keepSteerStill = false;
                break;
            }
        }

        for (int i = 0; i < modulesToApply.length; ++i) {

            states[i].angle = keepSteerStill
                    ? cachedModuleRotations[i]
                    : states[i].angle;

            cachedModuleRotations[i] = states[i].angle;

            modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
        }

        return StatusCode.OK;
    }

    public FieldCentricCorrect() {
        for (int i = 0; i < cachedModuleRotations.length; i++) {
            cachedModuleRotations[i] = new Rotation2d();
        }
    }

}
