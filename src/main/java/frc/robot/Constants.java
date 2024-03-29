// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.literals.CameraLiterals;
import frc.robot.literals.RobotLiterals;
import frc.robot.literals.SwerveDrivetrainLiterals;
import frc.robot.literals.TurretTalonConfigLiterals.CBOT_TurretTalonConfigLiterals;
import frc.robot.literals.TurretTalonConfigLiterals.PBOT_TurretTalonConfigLiterals;
import frc.robot.literals.TurretTalonConfigLiterals.TBOT_TurretTalonConfigLiterals;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static double LOOP_PERIOD_MILLIS = 20;

        /*
         * TBOT TBOT TBOT TBOT TBOT TBOT TBOT TBOT TBOT TBOT TBOT
         */

        static SwerveDrivetrainLiterals TBOT_SWERVE_LITERALS = new SwerveDrivetrainLiterals()
                        .withAutonomousMaxSpeed(3.0)
                        .withWheelRadius(3.9 / 2.0)
                        .withDriveGains(0.13, 1.16, 0.00715)
                        .withEncoderOffsets(-0.763916015625, -0.36962890625, -0.357421875, -0.5673828125)
                        .withRobotDimensions(23.0, 23.0);

        static CameraLiterals TBOT_CAMERA_LITERALS = new CameraLiterals( // TODO Remove TBOT, these settings are wrong
                                                                         // and probably wont use tbot anymore
                        Units.inchesToMeters(13.5),
                        0,
                        Units.inchesToMeters(8.75),
                        Rotation2d.fromDegrees(-7));

        /*
         * PBOT PBOT PBOT PBOT PBOT PBOT PBOT PBOT PBOT PBOT PBOT
         */

        static SwerveDrivetrainLiterals PBOT_SWERVE_LITERALS = new SwerveDrivetrainLiterals()
                        .withAutonomousMaxSpeed(3.0)
                        .withWheelRadius(3.9 / 2.0)
                        .withDriveGains(0.1866, 2.35554, 0.02379)
                        .withEncoderOffsets(0.00048828125, -0.481201171875, -0.2978515625, -0.37939453125)
                        .withRobotDimensions(21.5, 21.5);

        static CameraLiterals PBOT_CAMERA_LITERALS = new CameraLiterals(
                        Units.inchesToMeters(13.5),
                        0,
                        Units.inchesToMeters(8.75),
                        Rotation2d.fromDegrees(-7));

        /*
         * CBOT CBOT CBOT CBOT CBOT CBOT CBOT CBOT CBOT CBOT CBOT
         */

        static SwerveDrivetrainLiterals CBOT_SWERVE_LITERALS = new SwerveDrivetrainLiterals()
                        .withAutonomousMaxSpeed(3.0)
                        .withWheelRadius(3.9 / 2.0)
                        .withDriveGains(0.21346, 2.38051, 0.02546)
                        .withEncoderOffsets(-0.240722, 0.3996582, 0.32983, 0.23388)
                        .withRobotDimensions(21.5, 21.5);

        static CameraLiterals CBOT_CAMERA_LITERALS = new CameraLiterals(
                        Units.inchesToMeters(7.5),
                        0,
                        Units.inchesToMeters(8.75),
                        Rotation2d.fromDegrees(-13));

        private static RobotLiterals TBOT = new RobotLiterals(TBOT_SWERVE_LITERALS, TBOT_CAMERA_LITERALS,
                        new TBOT_TurretTalonConfigLiterals());

        private static RobotLiterals PBOT = new RobotLiterals(PBOT_SWERVE_LITERALS, PBOT_CAMERA_LITERALS,
                        new PBOT_TurretTalonConfigLiterals());

        private static RobotLiterals CBOT = new RobotLiterals(CBOT_SWERVE_LITERALS, CBOT_CAMERA_LITERALS,
                        new CBOT_TurretTalonConfigLiterals());

        public static RobotLiterals ROBOT = CBOT;

        public static final class Drive {

                public static final class ROBOT {
                        public static final double QUIESCENT_CURRENT_DRAW_A = 2.0; // Misc
                                                                                   // electronics
                        public static final double BATTERY_NOMINAL_VOLTAGE = 13.2; // Nicely
                                                                                   // charged
                                                                                   // battery
                                                                                   // 40mOhm - average batter + cabling
                        public static final double BATTERY_NOMINAL_RESISTANCE = 0.040;
                        public static final double MAX_VOLTAGE = 12.0; // Maximum Voltage
                                                                       // sent
                                                                       // to a motor
                                                                       // controller
                }

                public static final class AUTO {
                        public static final PathConstraints kPathConstraints = new PathConstraints(3.5, 6.0,
                                        Math.PI * 3, Math.PI * 6);
                        public static final PathConstraints kSlowPathConstraints = new PathConstraints(1.0, 6.0,
                                        Math.PI * 3, Math.PI * 6);
                }

                public static final class FIELD_DIMENSIONS {
                        public static final double LENGTH = Units.feetToMeters(54) + Units.inchesToMeters(3.25);
                        public static final double WIDTH = Units.feetToMeters(26) + Units.inchesToMeters(11.25);
                        public static final Translation2d CENTER_OF_FIELD = new Translation2d(LENGTH / 2.0,
                                        WIDTH / 2.0);

                        public static final Translation2d OFFSET_ALLIANCE_LINE_FROM_CENTER = new Translation2d(
                                        Units.inchesToMeters(95.4),
                                        0);
                        public static final Translation2d OFFSET_AUTO_CROSS_LINE_FROM_CENTER = new Translation2d(
                                        Units.inchesToMeters(250.50), 0);
                        public static final Translation2d OFFSET_CENTER_TO_SIDE_ROW_OF_NOTES = new Translation2d(
                                        Units.inchesToMeters(210.6), 0);

                        public static final Translation2d OFFSET_SIDE_NOTES = new Translation2d(0,
                                        Units.inchesToMeters(57.0));
                        public static final Translation2d OFFSET_CENTER_NOTES = new Translation2d(0,
                                        Units.inchesToMeters(66.0));
                }
        }

        public static final class EnabledDebugModes {
                public static final boolean CharacterizeEnabled = false;
                public static final boolean updatePoseWithVisionEnabled = true;
        }
}
