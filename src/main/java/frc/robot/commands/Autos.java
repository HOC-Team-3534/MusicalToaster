// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.Turret;
import swerve.CommandSwerveDrivetrain;

public final class Autos {

  public static Command getDynamicAutonomous(Pose2d initialPose, Turret turret, CommandSwerveDrivetrain drivetrain) {
    return Commands.runOnce(() -> drivetrain.seedFieldRelative(initialPose))
        .andThen(RobotContainer.getShootSpeakerCommand())
        .andThen(Commands.repeatingSequence(
        // First step is path follow to first note pose in hierarchy with intake on

        // If note is not there, go to next note in hierarchy
        // If you are shooting note, path follow to shoot location and shoot
        // If you are moving note shoot toward alliance wall on ground

        // repeat for all notes in hierarchy
        ));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
