package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
        private final double MaxSpeed;

        /**
         * Construct a telemetry object, with the specified max speed of the robot
         * 
         * @param maxSpeed
         *                 Maximum speed in meters per second
         */
        public Telemetry(double maxSpeed) {
                MaxSpeed = maxSpeed;
                SmartDashboard.putData("Field", m_field);
        }

        /* What to publish over networktables for telemetry */
        private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

        /* Robot speeds for general checking */
        private final NetworkTable driveStats = inst.getTable("Drive");
        private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
        private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
        private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();

        /* Keep a reference of the last pose to calculate the speeds */
        private Pose2d m_lastPose = new Pose2d();
        private double lastTime = Utils.getCurrentTimeSeconds();

        private final Field2d m_field = new Field2d();

        /* Mechanisms to represent the swerve module states */
        private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
                        new Mechanism2d(1, 1),
                        new Mechanism2d(1, 1),
                        new Mechanism2d(1, 1),
                        new Mechanism2d(1, 1),
        };
        /* A direction and length changing ligament for speed representation */
        private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
                        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
        };
        /* A direction changing and length constant ligament for module direction */
        private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
                        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                                                        new Color8Bit(Color.kWhite))),
                        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                                                        new Color8Bit(Color.kWhite))),
                        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                                                        new Color8Bit(Color.kWhite))),
                        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                                                        new Color8Bit(Color.kWhite))),
        };

        /* Accept the swerve drive state and telemeterize it to smartdashboard */
        public void telemeterize(SwerveDriveState state) {
                /* Telemeterize the pose */
                m_field.setRobotPose(state.Pose);

                /* Telemeterize the robot's general speeds */
                double currentTime = Utils.getCurrentTimeSeconds();
                double diffTime = currentTime - lastTime;
                lastTime = currentTime;
                Translation2d distanceDiff = state.Pose.minus(m_lastPose).getTranslation();
                m_lastPose = state.Pose;

                Translation2d velocities = distanceDiff.div(diffTime);

                speed.set(velocities.getNorm());
                velocityX.set(velocities.getX());
                velocityY.set(velocities.getY());

                /* Telemeterize the module's states */
                for (int i = 0; i < 4; ++i) {
                        m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
                        m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
                        m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

                        SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
                }
        }

        public Field2d getField() {
                return this.m_field;
        }
}
