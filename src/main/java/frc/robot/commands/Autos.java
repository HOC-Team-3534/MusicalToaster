// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.LinkedList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.stream.Collectors;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Drive.AUTO;
import frc.robot.Constants.Drive.FIELD_DIMENSIONS;
import frc.robot.RobotContainer.ShooterType;
import frc.robot.commands.AutoPosition.AutoPositionType;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.Turret;

public final class Autos {

  public static List<String> listAutoFiles() {
    try {
      // Define the path to the directory
      Path path = Paths.get(Filesystem.getDeployDirectory().getPath(), "pathplanner/autos");

      // Use Files.list to stream the files in the directory
      return Files.list(path)
          .filter(Files::isRegularFile) // Filter to include only regular files
          .map(Path::getFileName) // Get the file name
          .map(Path::toString) // Convert Path to String
          .filter(name -> name.endsWith(".auto")) // Filter to include only .auto files
          .map(name -> name.replace(".auto", "")) // Replace .auto with an empty string
          .collect(Collectors.toList()); // Collect names into a list
    } catch (IOException e) {
      throw new RuntimeException("Error reading auto files: " + e.getMessage(), e);
    }
  }

  public static Command getGUIAutoCommand(String autoName) {
    return new PathPlannerAuto(autoName);
  }

  static boolean pathsCompleted;
  static boolean goingToShootNoteFromCenter;
  static boolean justSkippedPathToShoot;

  public static Command getGUIAutoCommandNoNamedCommmands(LinkedList<PathPlannerPath> paths) {

    if (CommandSwerveDrivetrain.getInstance().isEmpty() || Turret.getInstance().isEmpty()
        || Intake.getInstance().isEmpty())
      return Commands.none();
    return CommandSwerveDrivetrain.getInstance().map((drivetrain) -> {
      pathsCompleted = false;
      justSkippedPathToShoot = false;
      currentPath = null;
      return (Command) Commands.deadline(
          /*
           * Step 1 :
           * 
           * Wait Until Note is Shot OR "going"/"heading" to shoot note closer to speaker
           * fromc center line
           * In order to be considering going to shoot the note from center line, the
           * robot must be too far way to shoot
           * AND the previous path can not have been skipped, meaning that the prior note
           * from center was not picked up.
           * 
           * NOTE: isValidShootPosition takes into account the robots velocity. If the
           * robot is moving and this is checked, it may cause issues. Right now, when
           * this is checked, the robot should not be following a path, so it should work
           * 
           * UPDATE: isValidShootPosition was not the problem, but for clarity, since we
           * only care about the position of the robot in this case, its better to have
           * a dedicated function to determine the region the robot is in for pathing
           * purposes. The real issue we had was IsNoteinRobot, because with a delayTimer
           * in Turret, it would pass right through the waitUntil, allowing the robot to
           * begin moving before it had shot the note it had becuase it thought it didn't
           * have a note yet.
           * 
           * 
           * Step 2 :
           * Drive and Intake. Skip if going to shoot Note from center but the robot did
           * not pick up the prior note from the center
           * 
           * Step 3 :
           * Wait for the robot to intake the note, unless the robot just shot the note it
           * picked up from the center because there is no note at the shoot position
           * 
           * This is repeated until all the paths are removed from the list of paths to
           * follow
           * A bit of time at the end is waited upon so that the robot can shoot the last
           * note
           * before the command ends because driving ended
           * 
           * Shooting and Intaking:
           * All the while driving,
           * intake is running if there is not a note in the robot,
           * shoot speaker is aiming and attempting to shoot if there is a note loaded in
           * the turret and the shoot position is valid, which accounts for robot speed
           * and it must be going slow or stopped in or to shoot
           */
          Commands
              .waitUntil(
                  () -> {
                    goingToShootNoteFromCenter = RobotState.isBeyondWing()
                        && !justSkippedPathToShoot;
                    return !RobotContainer.getRobotState().isNoteInRobot() || goingToShootNoteFromCenter;
                  })
              .andThen(getDrivePathCommand(paths).until(() -> {
                justSkippedPathToShoot = !RobotContainer.getRobotState().isNoteInRobot()
                    && goingToShootNoteFromCenter;
                return justSkippedPathToShoot;
              }),
                  Commands.waitUntil(() -> RobotContainer.getRobotState().isNoteInRobot() || goingToShootNoteFromCenter)
                      .withTimeout(1.0))
              .repeatedly()
              .until(() -> pathsCompleted)
              .andThen(Commands.waitSeconds(3.0)),
          RobotContainer.getShootCommand(() -> ShooterType.Speaker),
          RobotContainer.getIntakeAutonomouslyCommand());
    }).orElse(Commands.none());
  }

  static PathPlannerPath currentPath;

  public static PathPlannerPath getCurrentPath() {
    return currentPath;
  }

  private static Command getDrivePathCommand(LinkedList<PathPlannerPath> paths) {
    return Commands.deferredProxy(
        () -> {
          try {
            return (Command) CommandSwerveDrivetrain.getInstance()
                .map(drivetrain -> {
                  currentPath = paths.pop();
                  return drivetrain.followPath(currentPath);
                })
                .orElse(Commands.none());
          } catch (NoSuchElementException e) {
            pathsCompleted = true;
            currentPath = null;
            return Commands.none();
          }
        });
  }

  static Timer timer = new Timer();

  public static Command getDynamicAutonomous(AutoPositionList positions) {
    if (positions.isEmpty() || CommandSwerveDrivetrain.getInstance().isEmpty() || Turret.getInstance().isEmpty()
        || Intake.getInstance().isEmpty())
      return Commands.none();
    return CommandSwerveDrivetrain.getInstance().map((drivetrain) -> {
      return (Command) Commands.deadline(
          Commands.deferredProxy(
              () -> {
                timer.restart();
                return followPathToNextPositionCommand(drivetrain, positions)
                    .until(() -> timer.hasElapsed(0.25)
                        && !RobotContainer.getRobotState().isNoteInRobot()
                        && currentTargetAutoPosition.isSkippable()
                        && currentTargetAutoPosition.getType().equals(AutoPositionType.Shoot));
              })
              .alongWith(Commands.waitUntil(() -> !RobotContainer.getRobotState().isNoteInRobot()
                  || prevAutoPosition == null || prevAutoPosition.getType().equals(AutoPositionType.Shoot)))
              .repeatedly()
              .until(() -> positions.size() == 0 && currentTargetAutoPosition == null),
          // .andThen(Commands.waitSeconds(2.0)),
          RobotContainer.getShootCommand(
              () -> prevAutoPosition == null ? ShooterType.Speaker : prevAutoPosition.getShootOrStealNote()),
          RobotContainer.getIntakeAutonomouslyCommand());
    }).orElse(Commands.none());
  }

  public static AutoPosition currentTargetAutoPosition, prevAutoPosition;

  /**
   * Create the command to follow a path from the current position to the next
   * position
   * 
   * @param drivetrain
   * @return
   */
  private static Command followPathToNextPositionCommand(CommandSwerveDrivetrain drivetrain,
      LinkedList<AutoPosition> positions) {
    if (positions.isEmpty()) {
      currentTargetAutoPosition = null;
      return Commands.none();
    }

    prevAutoPosition = currentTargetAutoPosition;

    currentTargetAutoPosition = positions.pop();

    var current = drivetrain.getState().Pose.getTranslation();

    var constraints = current.getDistance(currentTargetAutoPosition.getPosition()) < Units.inchesToMeters(75.0)
        ? AUTO.kSlowPathConstraints
        : AUTO.kPathConstraints;

    return drivetrain.pathfindToPose(currentTargetAutoPosition.getPosition(), constraints, 0);
  }

  static final Translation2d OFFSET_STAGE_NOTE = new Translation2d(Units.inchesToMeters(10), 0);

  static Translation2d getNotePosition(int noteId) {
    var center = FIELD_DIMENSIONS.CENTER_OF_FIELD;

    var offsetStageNote = noteId == 3 || noteId == 11 ? OFFSET_STAGE_NOTE : new Translation2d();
    if (noteId >= 1 && noteId <= 3) { // BLUE
      return center.minus(FIELD_DIMENSIONS.OFFSET_CENTER_TO_SIDE_ROW_OF_NOTES.minus(offsetStageNote))
          .plus(FIELD_DIMENSIONS.OFFSET_SIDE_NOTES.times(3 - noteId));
    } else if (noteId >= 4 && noteId <= 8) { // CENTER
      return center.plus(FIELD_DIMENSIONS.OFFSET_CENTER_NOTES.times(6 - noteId));
    } else if (noteId >= 9 && noteId <= 11) {// RED
      return center.plus(FIELD_DIMENSIONS.OFFSET_CENTER_TO_SIDE_ROW_OF_NOTES.plus(offsetStageNote))
          .plus(FIELD_DIMENSIONS.OFFSET_SIDE_NOTES.times(11 - noteId));
    } else {
      return null;
    }
  }

  static final Translation2d OFFSET_UPPER_SHOOT_Y = FIELD_DIMENSIONS.OFFSET_CENTER_NOTES.plus(
      new Translation2d(0, Units.inchesToMeters(48.0)));
  static final Translation2d OFFSET_UPPER_SHOOT_X = FIELD_DIMENSIONS.OFFSET_ALLIANCE_LINE_FROM_CENTER
      .plus(new Translation2d(25.0, 0));
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

    public AutoPosition getNotePosition(ShooterType shootOrStealNote) {
      return new AutoPosition(Autos.getNotePosition(this.ordinal() + 1), AutoPositionType.Note, shootOrStealNote);
    }

    public AutoPosition getShootPostion(ShooterType shootOrStealNote) {
      var position = Autos.getShootPosition(this.ordinal() + 1);
      if (position != null)
        return new AutoPosition(position, AutoPositionType.Shoot, shootOrStealNote);
      return null;
    }

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
