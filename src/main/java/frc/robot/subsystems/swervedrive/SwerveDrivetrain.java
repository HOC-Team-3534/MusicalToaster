package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrivetrain extends MySwerveDrivetrain {
    private static final SendableChooser<String> centricityChooser = new SendableChooser<>();

    public enum RobotCentricity {
        FieldCentric, RobotCentric
    }

    public SwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants[] modules) {
        this(driveTrainConstants,
                OdometryUpdateFrequency,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.9, 0.9, 0.9),
                modules);
    }

    public SwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants[] modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
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

    public RobotCentricity getChosenRobotCentricity() {
        return centricityChooser.getSelected().equals(RobotCentricity.FieldCentric.toString())
                ? RobotCentricity.FieldCentric
                : RobotCentricity.RobotCentric;
    }

}