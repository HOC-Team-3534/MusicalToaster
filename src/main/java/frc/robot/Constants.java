// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
  public static RobotType ROBOTTYPE = RobotType.PBOT;
  public static double LOOP_PERIOD_MILLIS = 20;

  public enum RobotType {
    CBOT,
    PBOT,
    TBOT
  }

  public static final class Drive {
    public static final class Known {
      /**
       * The left-to-right distance between the drivetrain wheels
       * <p>
       * Should be measured from center to center.
       */
      public final static double TRACKWIDTH_METERS = (Constants.ROBOTTYPE == RobotType.TBOT) ? 0.5842
          : (Constants.ROBOTTYPE == RobotType.PBOT) ? 0.508 : 0.508;
      /**
       * The front-to-back distance between the drivetrain wheels.
       * <p>
       * Should be measured from center to center.
       */
      public final static double WHEELBASE_METERS = (Constants.ROBOTTYPE == RobotType.TBOT) ? 0.5842
          : (Constants.ROBOTTYPE == RobotType.PBOT) ? 0.508 : 0.5207;
      public static final double MAX_DRIVE_MOTOR_RPM = 6380.0; // Falcon500
                                                               // max rpm
    }

    public static final class Calculated {
      static final Translation2d FL_POS = new Translation2d(Drive.Known.TRACKWIDTH_METERS / 2.0,
          Drive.Known.WHEELBASE_METERS / 2.0);
      static final Translation2d FR_POS = new Translation2d(-FL_POS.getX(), FL_POS.getY());
      public static final double WHEELBASE_RADIUS = FL_POS.getNorm();
      public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(FL_POS, FR_POS,
          FR_POS.unaryMinus(), FL_POS.unaryMinus());
    }

    public static final class Config {
      public static final int PIGEON2_ID = 19;

      public static final class DriveCharacterization {
        public static final double QUASIASTIC_VOLTAGE = 1.0; // voltage
                                                             // per
                                                             // second
                                                             // increase
        public static final double QUASIASTIC_DURATION = 4.5; // duration
                                                              // of
                                                              // test
                                                              // seconds
      }
    }

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
      public static final Translation2d CENTER_OF_FIELD = new Translation2d(LENGTH / 2.0, WIDTH / 2.0);

      public static final Translation2d OFFSET_ALLIANCE_LINE_FROM_CENTER = new Translation2d(Units.inchesToMeters(95.4),
          0);
      public static final Translation2d OFFSET_AUTO_CROSS_LINE_FROM_CENTER = new Translation2d(
          Units.inchesToMeters(250.50), 0);
      public static final Translation2d OFFSET_CENTER_TO_SIDE_ROW_OF_NOTES = new Translation2d(
          Units.inchesToMeters(210.6), 0);

      public static final Translation2d OFFSET_SIDE_NOTES = new Translation2d(0, Units.inchesToMeters(57.0));
      public static final Translation2d OFFSET_CENTER_NOTES = new Translation2d(0, Units.inchesToMeters(66.0));
    }
  }

  public static final class EnabledDebugModes {
    public static final boolean CharacterizeEnabled = false;
    public static final boolean updatePoseWithVisionEnabled = false;
    public static final boolean testingTurret = false;
    public static final boolean calibratingTurret = false;
    public static final boolean isCalibratingOrTesting = calibratingTurret || testingTurret;
    public static final boolean testingClimber = false;
  }
}
