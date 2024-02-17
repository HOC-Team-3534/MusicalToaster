// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;
import java.util.LinkedList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.Drive.AUTO;
import frc.robot.Constants.Drive.FIELD_DIMENSIONS;
import frc.robot.commands.AutoPosition.AutoPositionType;
import frc.robot.subsystems.turret.Turret;
import swerve.CommandSwerveDrivetrain;

public final class Autos {

  public static Command getDynamicAutonomous(Turret turret, CommandSwerveDrivetrain drivetrain,
      AutoPositionList positions) {
    return Commands.runOnce(() -> drivetrain.seedFieldRelative(drivetrain.getState().Pose))
        .andThen(RobotContainer.getShootSpeakerCommand())
        .andThen(Commands
            .repeatingSequence(Commands.deferredProxy(() -> followPathToNextPositionCommand(drivetrain, positions)))
            .until(() -> positions.size() == 0));
  }

  /**
   * Create the command to follow a path from the current position to the next
   * position
   * 
   * @param drivetrain
   * @return
   */
  private static Command followPathToNextPositionCommand(CommandSwerveDrivetrain drivetrain,
      LinkedList<AutoPosition> positions) {
    if (positions.isEmpty())
      return Commands.none();

    var next = positions.pop();

    var current = drivetrain.getState().Pose.getTranslation();

    var constraints = current.getDistance(next.getPosition()) < Units.inchesToMeters(75.0)
        ? AUTO.kSlowPathConstraints
        : AUTO.kPathConstraints;

    return drivetrain.pathfindToPose(next.getPosition(), constraints, 0);
  }

  static final Translation2d OFFSET_CENTER_ROW_TO_SIDE = new Translation2d(Units.inchesToMeters(210.6), 0);
  static final Translation2d OFFSET_SIDE_NOTES = new Translation2d(0, Units.inchesToMeters(57.0));
  static final Translation2d OFFSET_CENTER_NOTES = new Translation2d(0, Units.inchesToMeters(66.0));

  static final Translation2d OFFSET_STAGE_NOTE = new Translation2d(Units.inchesToMeters(10), 0);

  static Translation2d getNotePosition(int noteId) {
    var center = FIELD_DIMENSIONS.CENTER_OF_FIELD;

    var offsetStageNote = noteId == 3 || noteId == 11 ? OFFSET_STAGE_NOTE : new Translation2d();
    if (noteId >= 1 && noteId <= 3) { // BLUE
      return center.minus(OFFSET_CENTER_ROW_TO_SIDE.minus(offsetStageNote))
          .plus(OFFSET_SIDE_NOTES.times(3 - noteId));
    } else if (noteId >= 4 && noteId <= 8) { // CENTER
      return center.plus(OFFSET_CENTER_NOTES.times(6 - noteId));
    } else if (noteId >= 9 && noteId <= 11) {// RED
      return center.plus(OFFSET_CENTER_ROW_TO_SIDE.plus(offsetStageNote))
          .plus(OFFSET_SIDE_NOTES.times(11 - noteId));
    } else {
      return null;
    }
  }

  static final Translation2d OFFSET_UPPER_SHOOT_Y = OFFSET_CENTER_NOTES.plus(
      new Translation2d(0, Units.inchesToMeters(48.0)));
  static final Translation2d OFFSET_UPPER_SHOOT_X = new Translation2d(
      FIELD_DIMENSIONS.OFFSET_ALLIANCE_LINE_FROM_CENTER + 25.0, 0);
  static final Translation2d OFFSET_LOWER_SHOOT_Y = OFFSET_UPPER_SHOOT_Y.times(1);
  static final Translation2d OFFSET_LOWER_SHOOT_X = OFFSET_UPPER_SHOOT_Y.times(1); // times 1 just clones it

  static Translation2d getShootPosition(int noteId) {
    if (noteId >= 4 && noteId <= 8) {
      var blue = DriverStation.getAlliance().get().equals(Alliance.Blue);
      var center = FIELD_DIMENSIONS.CENTER_OF_FIELD;
      if (noteId <= 6) { // upper
        return center.plus(OFFSET_UPPER_SHOOT_Y).plus(blue ? OFFSET_UPPER_SHOOT_X.unaryMinus() : OFFSET_UPPER_SHOOT_X);
      } else {
        return center.plus(OFFSET_LOWER_SHOOT_Y).plus(blue ? OFFSET_LOWER_SHOOT_X.unaryMinus() : OFFSET_LOWER_SHOOT_X);
      }
    } else {
      return null;
    }
  }

  public enum ShootOrStealNote {
    Shoot, Steal
  }

  public enum AutoNotes {
    BlueNote1,
    BlueNote2,
    BlueNote3,
    MiddleNote4,
    MiddleNote5,
    MiddleNote6,
    MiddleNote7,
    MiddleNote8,
    RedNote9,
    RedNote10,
    RedNote11;

    ShootOrStealNote shootOrStealNote;

    public AutoPosition getNotePosition() {
      return new AutoPosition(Autos.getNotePosition(this.ordinal() + 1), AutoPositionType.Note);
    }

    public AutoPosition getShootPostion() {
      var position = Autos.getShootPosition(this.ordinal() + 1);
      if (position != null)
        return new AutoPosition(position, AutoPositionType.Shoot);
      return null;
    }

    public ShootOrStealNote getShootOrStealNote() {
      return this.shootOrStealNote;
    }

    public AutoNotes withShootOrStealNote(ShootOrStealNote shootOrStealNote) {
      this.shootOrStealNote = shootOrStealNote;
      return this;
    }

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
