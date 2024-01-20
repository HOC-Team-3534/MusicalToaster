package frc.robot;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.AUTO;
import swerve.SDSModuleConfiguration;
import swerve.params.PoseEstimationStandardDeviations;
import swerve.params.RobotMaxKinematics;
import swerve.params.SwerveParams;

public class SwerveHelper {
        public static SwerveParams loadSwerveParams() {
                var driveSlotConfigs = new SlotConfigs();
                switch (Constants.ROBOTTYPE) {
                        case CBOT:
                                driveSlotConfigs.withKS(0.31).withKV(2.382).withKA(0.031);
                                break;
                        case PBOT:
                                driveSlotConfigs.withKS(0.31).withKV(2.382).withKA(0.031);
                                break;
                        case TBOT:
                                driveSlotConfigs.withKS(0.293).withKV(2.367).withKA(0.0379);
                                break;
                        default:
                                break;

                }
                driveSlotConfigs.kS /= 12;
                driveSlotConfigs.kV /= 12;
                driveSlotConfigs.kA /= 12;
                var config = Constants.Drive.Known.SDS_MODULE_CONFIGURATION;
                var newConfig = new SDSModuleConfiguration(Units.inchesToMeters(3.81),
                                config.angleGearRatio,
                                config.driveGearRatio,
                                config.angleSlotConfigs,
                                config.driveMotorInvert,
                                config.angleMotorInvert,
                                config.canCoderSensorDirection);

                var modulePoseEstStdDev = new PoseEstimationStandardDeviations(0.1, 0.1, Rotation2d.fromDegrees(0.01));
                var visionPoseEstStdDev = new PoseEstimationStandardDeviations(0.15, 0.15,
                                Rotation2d.fromDegrees(0.25));
                var maxKinematics = new RobotMaxKinematics(Drive.Calculated.MAX_FWD_REV_SPEED_MPS_EST,
                                Drive.Calculated.MAX_ROTATE_SPEED_RAD_PER_SEC_EST, 2 * Math.PI);
                var holoConfig = new HolonomicPathFollowerConfig(new PIDConstants(10), new PIDConstants(10),
                                AUTO.kPathConstraints.getMaxVelocityMps(), Drive.Calculated.WHEELBASE_RADIUS,
                                new ReplanningConfig());

                return new SwerveParams(maxKinematics, Drive.Calculated.KINEMATICS, newConfig, holoConfig,
                                driveSlotConfigs,
                                modulePoseEstStdDev,
                                visionPoseEstStdDev);
        }
}
