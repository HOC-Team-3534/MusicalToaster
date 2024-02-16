// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.LinkedList;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoPosition.AutoPositionType;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.turret.Turret;
import swerve.CommandSwerveDrivetrain;

public final class Autos {

  public static Command getDynamicAutonomous(Pose2d initialPose, Turret turret, CommandSwerveDrivetrain drivetrain,
      AutoPositions... positions) {
    Autos.positions = new LinkedList<AutoPosition>();

    Autos.positions.add(new AutoPosition(initialPose.getTranslation(), AutoPositionType.Shoot));
    for (AutoPositions p : positions) {
      Autos.positions.add(p.getNotePosition());
      var shoot = p.getShootPostion();
      if (shoot != null) {
        Autos.positions.add(shoot);
      }
    }
    return Commands.runOnce(() -> drivetrain.seedFieldRelative(initialPose))
        .andThen(RobotContainer.getShootSpeakerCommand())
        .andThen(Commands.repeatingSequence(Commands.deferredProxy(() -> Autos.getCurrentPath(drivetrain)))
            .until(() -> Autos.positions.size() == 0)
        // drivetrain.pathfindToPose(new Pose2d(positions[0],n
        // Rotation2d()),pathConstraints,new GoalEndState(0, null))--Need to calculate
        // approach heading
        // First step is path follow to first note pose in hierarchy with intake on
        // If note is not there, go to next note in hierarchy
        // If you are shooting note, path follow to shoot location and shoot
        // If you are moving note shoot toward alliance wall on ground

        // repeat for all notes in hierarchy
        );
  }

  private static LinkedList<AutoPosition> positions;

  public static Command getCurrentPath(CommandSwerveDrivetrain drivetrain) {
    var current = positions.pop();
    var next = positions.peek();
    double endVelocity = 0;
    if (next == null) {
      positions.pop();
      return Commands.none();
    }

    var currentDelta = next.getPosition().minus(current.getPosition());
    var currentDeltaX = currentDelta.getX();
    var nextDeltaX = positions.get(1).getPosition().minus(next.getPosition()).getX();
    if (Math.abs(nextDeltaX) < 0.05) {
      endVelocity = slowVelocity;
    }

    var maxSpeed = getPathConstraints().getMaxVelocityMps();
    if (Math.abs(currentDeltaX) < 0.05) {
      maxSpeed = slowVelocity;
    }
    return drivetrain.pathfindToPose(next.getPosition(), getPathConstraints(maxSpeed),
        endVelocity);

  }

  public static double slowDist = 2.0, slowVelocity = 1.0;

  public static PathConstraints getPathConstraints() {
    return getPathConstraints(TunerConstants.kAutonomousMaxSpeedMps);
  }

  public static PathConstraints getPathConstraints(double maxSpeed) {
    return new PathConstraints(maxSpeed, 6.0,
        Math.PI * 3, Math.PI * 6);
  }

  public static Command placeHolderForPathFind(Translation2d targetPose, PathConstraints constraints,
      double goalEndVelocity) {
    return null;
  }

  public static final double centerRowX = Units.inchesToMeters(324.6);
  public static final double blueRowX = Units.inchesToMeters(centerRowX - 210.6);
  public static final double redRowX = Units.inchesToMeters(centerRowX + 210.6);
  public static final double centerOfFieldNotes = Units.inchesToMeters(161.64);
  public static final double offsetOfSideNotes = Units.inchesToMeters(57.0);
  public static final double offsetOfMiddleNotes = Units.inchesToMeters(66.0);

  public enum AutoPositions {
    BlueNote1(new Translation2d(Autos.blueRowX, Autos.centerOfFieldNotes + (Autos.offsetOfSideNotes * 2)),
        null),
    BlueNote2(new Translation2d(Autos.blueRowX, Autos.centerOfFieldNotes + (Autos.offsetOfSideNotes)),
        null),
    BlueNote3(new Translation2d(Autos.blueRowX, Autos.centerOfFieldNotes), null),
    MiddleNote4(new Translation2d(Autos.centerRowX, Autos.centerOfFieldNotes + (Autos.offsetOfMiddleNotes * 2)),
        null),
    MiddleNote5(new Translation2d(Autos.centerRowX, Autos.centerOfFieldNotes + (Autos.offsetOfMiddleNotes)),
        null),
    MiddleNote6(new Translation2d(Autos.centerRowX, Autos.centerOfFieldNotes), null),
    MiddleNote7(new Translation2d(Autos.centerRowX, Autos.centerOfFieldNotes - (Autos.offsetOfMiddleNotes)),
        null),
    MiddleNote8(new Translation2d(Autos.centerRowX, Autos.centerOfFieldNotes - (Autos.offsetOfMiddleNotes * 2)),
        null),
    RedNote9(new Translation2d(Autos.redRowX, Autos.centerOfFieldNotes + (Autos.offsetOfSideNotes * 2)),
        null),
    RedNote10(new Translation2d(Autos.redRowX, Autos.centerOfFieldNotes + (Autos.offsetOfSideNotes)),
        null),
    RedNote11(new Translation2d(Autos.redRowX, Autos.centerOfFieldNotes), null);

    public AutoPosition note, shoot;

    AutoPositions(Translation2d noteTranslation, Translation2d shootTranslation) {
      this.note = new AutoPosition(noteTranslation, AutoPositionType.Note);
      this.shoot = shootTranslation == null ? null : new AutoPosition(shootTranslation, AutoPositionType.Shoot);

    }

    public AutoPosition getNotePosition() {
      return this.note;
    }

    public AutoPosition getShootPostion() {
      return this.shoot;
    }

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
