package frc.robot.subsystems.swervedrive;

import org.opencv.core.Point;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrivetrain extends com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain {
    private static final SendableChooser<String> centricityChooser = new SendableChooser<>();

    public enum RobotCentricity {
        FieldCentric, RobotCentric
    }

    public SwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants[] modules) {
        super(driveTrainConstants, 100, odometryStandardDeviation, visionStandardDeviation,
                modules);
        centricityChooser.setDefaultOption("Field Centric", RobotCentricity.FieldCentric.toString());
        centricityChooser.addOption("Robot Centric", RobotCentricity.RobotCentric.toString());
        SmartDashboard.putData("Centricity Chooser", centricityChooser);
    }

    /**
     * Gets the kinematics of the drivetrain.
     *
     * @return Kinematics of the drivetrain
     */
    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.getKinematics().toChassisSpeeds(this.getState().ModuleStates);
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        var speeds = getChassisSpeeds();
        var rotated = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                .rotateBy(this.getState().Pose.getRotation());
        return new ChassisSpeeds(rotated.getX(), rotated.getY(), speeds.omegaRadiansPerSecond);
    }

    public RobotCentricity getChosenRobotCentricity() {
        return centricityChooser.getSelected().equals(RobotCentricity.FieldCentric.toString())
                ? RobotCentricity.FieldCentric
                : RobotCentricity.RobotCentric;
    }

}