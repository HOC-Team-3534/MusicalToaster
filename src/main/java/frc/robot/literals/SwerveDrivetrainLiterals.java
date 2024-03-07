package frc.robot.literals;

import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.path.IPathPlanner;

public class SwerveDrivetrainLiterals {

        public SwerveDrivetrainLiterals withEncoderOffsets(
                        double frontLeft,
                        double frontRight,
                        double backLeft,
                        double backRight) {
                this.kFrontLeftEncoderOffset = frontLeft;
                this.kFrontRightEncoderOffset = frontRight;
                this.kBackLeftEncoderOffset = backLeft;
                this.kBackRightEncoderOffset = backRight;
                return this;
        }

        public SwerveDrivetrainLiterals withDriveGains(double kS, double kV, double kA) {
                driveGains.withKS(kS).withKV(kV).withKA(kA);
                return this;
        }

        public SwerveDrivetrainLiterals withAutonomousMaxSpeed(double mps) {
                this.kAutonomousMaxSpeedMps = mps;
                return this;
        }

        public SwerveDrivetrainLiterals withCANbusName(String name) {
                this.kCANbusName = name;
                return this;
        }

        public SwerveDrivetrainLiterals withRobotDimensions(double frontToBackInches, double leftToRightInches) {
                this.kfrontToBackInches = frontToBackInches;
                this.kleftToRightInches = leftToRightInches;
                return this;
        }

        public SwerveDrivetrainLiterals withWheelRadius(double radiusInches) {
                this.kWheelRadiusInches = radiusInches;
                return this;
        }

        public SwerveDrivetrainLiterals withPathPlanner(IPathPlanner pathPlanner) {
                this.pathPlanner = pathPlanner;
                return this;
        }

        private Slot0Configs driveGains = new Slot0Configs()
                        .withKP(3).withKI(0).withKD(0);

        private double kAutonomousMaxSpeedMps;
        private double kWheelRadiusInches;

        private String kCANbusName = "Swerve";

        private double kfrontToBackInches, kleftToRightInches;

        private double kFrontLeftEncoderOffset,
                        kFrontRightEncoderOffset,
                        kBackLeftEncoderOffset,
                        kBackRightEncoderOffset;

        private IPathPlanner pathPlanner = new IPathPlanner.PathPlanner2024_FixedFromGUI();
        /**
         * Static
         */

        private static final double kMaxRPMofFalcon = 6380.0;

        private static final Slot0Configs steerGains = new Slot0Configs()
                        .withKP(100).withKI(0).withKD(0.2)
                        .withKS(0).withKV(1.5).withKA(0);

        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        private static final double kSlipCurrentA = 300.0;
        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.746031746031747;
        private static final double kSteerGearRatio = 12.8;

        private static final boolean kSteerMotorReversed = false;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        private static final int kPigeonId = 13;

        enum ModuleCorner {
                FL, FR, BL, BR;

                final int driveId;
                final int steerId;
                final int cancoderId;

                ModuleCorner() {
                        this.driveId = this.ordinal() * 3 + 1;
                        this.cancoderId = this.ordinal() * 3 + 2;
                        this.steerId = this.ordinal() * 3 + 3;
                }

                public Translation2d getXY(double frontToBackInches, double leftToRightInches) {
                        var position = new Translation2d(frontToBackInches / 2.0, leftToRightInches / 2.0)
                                        .times(Units.inchesToMeters(1));
                        switch (this) {
                                case FL:
                                        return position;
                                case FR:
                                        return new Translation2d(position.getX(), -position.getY());
                                case BL:
                                        return new Translation2d(-position.getX(), position.getY());
                                case BR:
                                        return position.unaryMinus();
                                default:
                                        return null;
                        }

                }
        }

        private SwerveModuleConstantsFactory getConstantsCreator() {
                return new SwerveModuleConstantsFactory()
                                .withDriveMotorGearRatio(kDriveGearRatio)
                                .withSteerMotorGearRatio(kSteerGearRatio)
                                .withWheelRadius(kWheelRadiusInches)
                                .withSlipCurrent(kSlipCurrentA)
                                .withSteerMotorGains(steerGains)
                                .withDriveMotorGains(driveGains)
                                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                                .withSpeedAt12VoltsMps(getMaxSpeedAt12V())
                                .withSteerInertia(kSteerInertia)
                                .withDriveInertia(kDriveInertia)
                                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                                .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
                                .withCouplingGearRatio(kCoupleRatio)
                                .withSteerMotorInverted(kSteerMotorReversed);
        }

        private SwerveModuleConstants createModuleConstants(SwerveModuleConstantsFactory ConstantCreator,
                        ModuleCorner moduleCorner,
                        double encoderOffsetRotations,
                        boolean invert) {

                var position = moduleCorner.getXY(kfrontToBackInches, kleftToRightInches);

                return ConstantCreator.createModuleConstants(
                                moduleCorner.steerId,
                                moduleCorner.driveId,
                                moduleCorner.cancoderId,
                                encoderOffsetRotations,
                                position.getX(),
                                position.getY(),
                                invert);

        }

        public Optional<CommandSwerveDrivetrain> getDrivetrain() {
                var DrivetrainConstants = new SwerveDrivetrainConstants()
                                .withPigeon2Id(kPigeonId)
                                .withCANbusName(kCANbusName);

                var ConstantCreator = getConstantsCreator();

                var FrontLeft = createModuleConstants(
                                ConstantCreator, ModuleCorner.FL, kFrontLeftEncoderOffset, kInvertLeftSide);
                var FrontRight = createModuleConstants(
                                ConstantCreator, ModuleCorner.FR, kFrontRightEncoderOffset, kInvertRightSide);
                var BackLeft = createModuleConstants(
                                ConstantCreator, ModuleCorner.BL, kBackLeftEncoderOffset, kInvertLeftSide);
                var BackRight = createModuleConstants(
                                ConstantCreator, ModuleCorner.BR, kBackRightEncoderOffset, kInvertRightSide);

                var rotateRadius = new Translation2d(kfrontToBackInches / 2.0, kleftToRightInches / 2.0)
                                .times(Units.inchesToMeters(1)).getNorm();

                var HoloConfig = new HolonomicPathFollowerConfig(
                                new PIDConstants(1), new PIDConstants(1),
                                getMaxSpeedAt12V(), rotateRadius,
                                new ReplanningConfig());

                return CommandSwerveDrivetrain.createInstance(DrivetrainConstants,
                                VecBuilder.fill(0.1, 0.1, 0.1),
                                VecBuilder.fill(0.1, 0.1, 0.1),
                                getMaxSpeedAt12V(),
                                HoloConfig,
                                pathPlanner,
                                FrontLeft,
                                FrontRight, BackLeft, BackRight);
        }

        public double getMaxSpeedAt12V() {
                return kMaxRPMofFalcon / 60.0
                                / kDriveGearRatio * Units.inchesToMeters(kWheelRadiusInches) * 2 * Math.PI;
        }

        public double getMaxSpeedAutonomous() {
                return this.kAutonomousMaxSpeedMps;
        }

        public IPathPlanner getPathPlanner() {
                return this.pathPlanner;
        }

}
