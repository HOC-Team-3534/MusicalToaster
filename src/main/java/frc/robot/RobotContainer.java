// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.LinkedList;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drive.FIELD_DIMENSIONS;
import frc.robot.ControllerInputs.AXS;
import frc.robot.ControllerInputs.BTN;
import frc.robot.ControllerInputs.TGR;
import frc.robot.commands.AutoPosition;
import frc.robot.commands.AutoPosition.AutoPositionType;
import frc.robot.commands.AutoPositionList;
import frc.robot.commands.Autos;
import frc.robot.subsystems.camera.PhotonVisionCamera;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberRequest;
import frc.robot.subsystems.climber.ClimberRequest.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRequest.ControlIntake;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightModes;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.path.IPathPlanner;
import frc.robot.subsystems.turret.ShooterRequest;
import frc.robot.subsystems.turret.ShooterRequest.*;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretRequest;
import frc.robot.subsystems.turret.TurretRequest.*;
import frc.robot.utils.PathUtils;
import frc.robot.utils.TelemetryUtils;
import frc.robot.utils.auto.SendableChooserWithChangeDetector;
import frc.robot.utils.swerve.FieldCentricCorrect;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	// The driver station connected controllers are defined here...
	private static double MaxAngularRate = 4 * Math.PI;

	private final static FieldCentric drive = new FieldCentricCorrect();

	private final static ControlClimber climb = new ControlClimber().withVoltage(12.0);
	private final static ResetClimber resetClimber = new ResetClimber();
	private final static ClimberRequest.Idle climberOff = new ClimberRequest.Idle();

	private final static ControlIntake runIntake = new ControlIntake().withIntakePercent(1.0);
	private final static ControlIntake runExtake = new ControlIntake().withIntakePercent(-1.0);
	private final static ControlIntake stopIntake = new ControlIntake().withIntakePercent(0.0);

	private final static IndexFromIntake indexFromIntake = new IndexFromIntake()
			.withRollerOutput(-1.0)
			.withTilt(Rotation2d.fromDegrees(40));
	private final static ControlTurret aimForSpeaker = new ControlTurret()
			.withTargetAzimuthFunction(turretState -> RobotState.getPoseRotation()
					.flatMap(robotRotation -> turretState.getVirtualGoalLocationDisplacement()
							.map(displacementToGoal -> displacementToGoal.getAngle().minus(robotRotation))))
			.withTargetElevationFunction(
					(turretState) -> turretState.getVirtualGoalLocationDisplacement().map((displacementToGoal) -> {
						var distance = displacementToGoal.getNorm();
						var a = 1.93651;
						var b = -18.6632;
						var c = 68.975;
						var degrees = a * Math.pow(distance, 2) + b * distance + c;
						SmartDashboard.putNumber("Distance from Goal", distance);
						return Rotation2d.fromDegrees(degrees);
					}))
			.withAllowShootWhenAimedSupplier(() -> RobotState.isValidShootPosition());

	private final static ControlTurret aimSubwoofer = new ControlTurret()
			.withTargetAzimuthFunction(turretState -> BTN.ManualShooting.get()
					? Optional.of(new Rotation2d())
					: RobotState.getPoseRotation()
							.flatMap(robotRotation -> turretState.getVirtualGoalLocationDisplacement()
									.map(displacementToGoal -> displacementToGoal.getAngle().minus(robotRotation))))
			.withTargetElevationFunction((turretState) -> {
				var degrees = SmartDashboard.getNumber("Tilt", 49);
				return Optional.of(Rotation2d.fromDegrees(degrees));
			})
			.withAllowShootWhenAimedSupplier(() -> BTN.SubwooferLetItRip.get());
	private final static ControlTurret aimForSteal = new ControlTurret()
			.withTargetAzimuthFunction((turretState) -> RobotState.getPoseRotation()
					.flatMap(robotRotation -> DriverStation.getAlliance().map(alliance -> {
						var targetAzimuth = alliance.equals(Alliance.Blue)
								? Rotation2d.fromDegrees(180)
								: new Rotation2d();
						return targetAzimuth.minus(robotRotation);
					})))
			.withTargetElevationFunction((turretState) -> Optional.of(Rotation2d.fromDegrees(10)));
	private final static JustRoller reloadNote = new JustRoller()
			.withRollerPercent(0.5)
			.withTilt(Rotation2d.fromDegrees(40))
			.withTurnOnRollerSupplier((withinTolerance) -> BTN.ReloadNoteActivate.get());
	private final static ControlTurret prepareForClimbTurret = new ControlTurret()
			.withTargetAzimuthFunction((turretState) -> Optional.of(Rotation2d.fromDegrees(90)))
			.withTargetElevationFunction((turretState) -> Optional.of(new Rotation2d()))
			.withAllowShootWhenAimedSupplier(() -> false);
	private final static ControlTurret extakeTurret = new ControlTurret()
			.withTargetAzimuthFunction((turretState) -> Optional.of(turretState.getCurrentAzimuth()))
			.withTargetElevationFunction(turretState -> Optional.of(Rotation2d.fromDegrees(-20.0)));

	private final static ShooterRequest.Idle shooterOff = new ShooterRequest.Idle();
	private final static ControlShooter shootSubwoofer = new ControlShooter().withVelocity(80);// TODO Find these
	private final static ControlShooter shooterSpeaker = new ControlShooter().withVelocity(80);// TODO Find these
	private final static ControlShooter shooterSteal = new ControlShooter().withVelocity(15);
	private final static ControlShooter shooterExtake = new ControlShooter().withVelocity(15);

	private static SendableChooserWithChangeDetector<Autos.AutoNotes>[] noteHiearchyChoosers = new SendableChooserWithChangeDetector[5];
	private static SendableChooserWithChangeDetector<ShooterType>[] shootOrStealChoosers = new SendableChooserWithChangeDetector[5];

	private static SendableChooserWithChangeDetector<String> guiAutoChooser = new SendableChooserWithChangeDetector<>();
	private static SendableChooserWithChangeDetector<Integer> maxAutoPathsChooser = new SendableChooserWithChangeDetector<>();

	private static RobotState m_robotState = new RobotState();

	public static RobotState getRobotState() {
		return m_robotState;
	}

	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public static void initialize() {
		// instantiate subsystems must be called first
		// TODO Move to Robot.java?
		instantiateSubsystems();

		defineDefaultCommands();

		// Configure the trigger bindings
		configureBindings();

		SmartDashboard.putNumber("Tilt", 49.0); // TODO Move somewhere else?

		if (Utils.isSimulation()) { // TODO Do we need to set the pose if simulation if we arent really focused on
									// simulation?
			CommandSwerveDrivetrain.getInstance().ifPresent((drivetrain) -> drivetrain
					.seedFieldRelative(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90))));
		}

		createAutonomousChoosers();

		if (Constants.ROBOT.getPathPlanner() instanceof IPathPlanner.PathPlanner2024_FixedFromGUI)
			registerNamedCommands();
	}

	private static void instantiateSubsystems() {
		Constants.ROBOT.getDrivetrain();
		Turret.createInstance();
		Intake.createInstance();
		if (RobotBase.isReal())
			PhotonVisionCamera.createInstance();
		Climber.createInstance();
		Lights.createInstance();
	}

	private static void registerNamedCommands() {
		var shootCommand = getShootCommand(() -> ShooterType.Speaker).until(() -> !getRobotState().isNoteInRobot());
		var intakeCommand = getIntakeAutonomouslyCommand().until(() -> getRobotState().isNoteLoaded());

		NamedCommands.registerCommand("ShootSpeakerUntilNotLoaded", shootCommand);
		NamedCommands.registerCommand("IntakeUntilLoaded", intakeCommand);
	}

	private static void defineDefaultCommands() {
		CommandSwerveDrivetrain.getInstance().ifPresent((drivetrain) -> {
			drivetrain.setDefaultCommand(
					drivetrain.applyRequest(
							() -> drive
									.withVelocityX(
											AXS.Drive_ForwardBackward.getAxis()
													* Constants.ROBOT.getMaxSpeedat12V())
									.withVelocityY(
											AXS.Drive_LeftRight.getAxis()
													* Constants.ROBOT.getMaxSpeedat12V())
									.withRotationalRate(
											AXS.Drive_Rotation.getAxis() * MaxAngularRate)));
		});

		Intake.getInstance().ifPresent((intake) -> intake.setDefaultCommand(
				intake.applyRequest(
						() -> getRobotState().isActivelyGrabbing()
								? runIntake
								: stopIntake)));

		Turret.getInstance().ifPresent((turret) -> turret.setDefaultCommand(
				turret.applyRequest(() -> getRobotState().isClimbing() ? prepareForClimbTurret : indexFromIntake,
						() -> shooterOff)));

		Climber.getInstance().ifPresent(climber -> climber.setDefaultCommand(climber.applyRequest(() -> climberOff)));
	}

	public static void createAutonomousChoosers() {

		if (Constants.ROBOT.getPathPlanner() instanceof IPathPlanner.PathPlanner2024) {
			for (int i = 0; i < 5; i++) {
				noteHiearchyChoosers[i] = newNoteHiearchyChooser();
				SmartDashboard.putData("Note Hiearchy " + (i + 1), noteHiearchyChoosers[i]);
				shootOrStealChoosers[i] = newShootOrStealChooser();
				SmartDashboard.putData("Shoot|Steal " + (i + 1), shootOrStealChoosers[i]);
			}
		} else {
			guiAutoChooser.setDefaultOption("DriveToBottom",
					"DriveToBottom");
			for (String autoName : Autos.listAutoFiles()) {
				guiAutoChooser.addOption(autoName, autoName);
			}
			SmartDashboard.putData("GUI Path Planner Auto", guiAutoChooser);

			maxAutoPathsChooser.setDefaultOption("20", 20);
			for (int i = 1; i <= 20; i++) {
				maxAutoPathsChooser.addOption(i + "", i);
			}
			SmartDashboard.putData("GUI Auto Max Paths To Follow", maxAutoPathsChooser);
		}
	}

	public static SendableChooserWithChangeDetector<Autos.AutoNotes> newNoteHiearchyChooser() {
		SendableChooserWithChangeDetector<Autos.AutoNotes> noteHiearchy = new SendableChooserWithChangeDetector<>();

		noteHiearchy.setDefaultOption("None", null);
		for (Autos.AutoNotes note : Autos.AutoNotes.values()) {
			noteHiearchy.addOption(note.name(), note);
		}

		return noteHiearchy;
	}

	public static SendableChooserWithChangeDetector<ShooterType> newShootOrStealChooser() {
		SendableChooserWithChangeDetector<ShooterType> shootOrSteal = new SendableChooserWithChangeDetector<>();

		shootOrSteal.setDefaultOption("Shoot", ShooterType.Speaker);
		shootOrSteal.addOption("Steal", ShooterType.Steal);

		return shootOrSteal;
	}

	private static void configureBindings() {
		TGR.Characterize.tgr().whileTrue(CommandSwerveDrivetrain.getInstance()
				.map((drivetrain) -> drivetrain.characterizeDrive(1.0, 4.0))
				.orElse(Commands.none()));

		TGR.Intake.tgr()
				.whileTrue(Intake.getInstance()
						.map((intake) -> intake
								.applyRequest(
										() -> !getRobotState().isNoteInRobot() || getRobotState().isActivelyGrabbing()
												? runIntake
												: stopIntake))
						.orElse(Commands.none()));
		TGR.Extake.tgr()
				.whileTrue(Commands.parallel(Intake.getInstance()
						.map((intake) -> intake.applyRequest(() -> runExtake))
						.orElse(Commands.none()),
						Commands.run(() -> {
							Intake.getInstance().ifPresent((intake) -> intake.resetAllNoteInPosition());
							Turret.getInstance().ifPresent(turret -> turret.resetNoteLoaded());
							getRobotState().setExtaking();
						}), getShootCommand(() -> ShooterType.ExtakeFromTurret)))
				.onFalse(Commands.runOnce(() -> getRobotState().resetExtaking()));

		TGR.ShootSpeaker.tgr().whileTrue(getShootCommand(() -> ShooterType.Speaker));
		TGR.PrepareShootForSubwoofer.tgr().whileTrue(getShootCommand(() -> ShooterType.Subwoofer));

		TGR.ReloadNote.tgr().whileTrue(getShootCommand(() -> ShooterType.ReloadNote));

		TGR.ResetNoteinRobot.tgr().onTrue(Commands.runOnce(() -> {
			getRobotState().setGrabNoteIndex(-1);
			getRobotState().setNoteLoaded(false);
			Intake.getInstance().ifPresent(intake -> intake.resetAllNoteInPosition());
			Turret.getInstance().ifPresent(turret -> turret.resetNoteLoaded());
		}));

		TGR.Climb.tgr().whileTrue(
				Climber.getInstance().map(climber -> climber.applyRequest(() -> {
					getRobotState().setClimbing();
					return climb;
				})).orElse(Commands.none()));

		TGR.ResetClimber.tgr().whileTrue(
				Climber.getInstance().map(climber -> climber.applyRequest(() -> resetClimber)).orElse(Commands.none()));

		new Trigger(() -> getRobotState().isNoteInRobot()).whileTrue(Lights.getInstance().map(lights -> {
			return lights.applyLightMode(LightModes.SolidGreen);
		}).orElse(Commands.none()));

		TGR.AmpLights.tgr()
				.whileTrue(Lights.getInstance().map(lights -> lights.applyLightMode(LightModes.SolidYellow))
						.orElse(Commands.none()));

		// TGR.HoldManualAimSubwoofer.tgr()
		// .toggleOnTrue(Commands.runOnce(() ->
		// getRobotState().isManualAimingSubwoofer()));

	}

	public static Command getIntakeAutonomouslyCommand() {
		return Intake.getInstance()
				.map((intake) -> intake
						.applyRequest(
								() -> !getRobotState().isNoteInRobot() || getRobotState().isActivelyGrabbing()
										? runIntake
										: stopIntake))
				.orElse(Commands.none());
	}

	public enum ShooterType {
		Speaker(aimForSpeaker, shooterSpeaker),
		Steal(aimForSteal, shooterSteal),
		Subwoofer(aimSubwoofer, shootSubwoofer),
		ReloadNote(reloadNote, shooterOff),
		ExtakeFromTurret(extakeTurret, shooterExtake);

		TurretRequest turretRequest;
		ShooterRequest shooterRequest;

		ShooterType(TurretRequest turretRequest, ShooterRequest shooterRequest) {
			this.turretRequest = turretRequest;
			this.shooterRequest = shooterRequest;
		}
	}

	public static Command getShootCommand(Supplier<ShooterType> shooterTypeSupplier) {
		return Turret.getInstance().map((turret) -> {
			var type = shooterTypeSupplier.get();
			return turret.applyRequest(
					() -> getRobotState().noteLoaded || type.equals(ShooterType.ReloadNote)
							|| type.equals(ShooterType.ExtakeFromTurret) ? type.turretRequest
									: indexFromIntake,
					() -> type.shooterRequest);
		}).orElse(Commands.none());
	}

	public static Command getAutonomousCommand() {
		if (Constants.ROBOT.getPathPlanner() instanceof IPathPlanner.PathPlanner2024)
			return getAutonomousCommandDynamic();

		return getAutonomousCommand_FixedFromGUI();
	}

	private static Command getAutonomousCommand_FixedFromGUI() {
		Turret.getInstance().ifPresent(turret -> turret.setNoteLoadedNoDelay());
		var paths = PathUtils.getAutoPaths(guiAutoChooser.getSelected(), maxAutoPathsChooser.getSelected());
		return Autos.getGUIAutoCommandNoNamedCommmands(paths);
	}

	static final Translation2d DriveStraightForwardLine = FIELD_DIMENSIONS.CENTER_OF_FIELD
			.minus(FIELD_DIMENSIONS.OFFSET_AUTO_CROSS_LINE_FROM_CENTER)
			.plus(new Translation2d(Units.inchesToMeters(25.0), 0));

	private static Command getAutonomousCommandDynamic() {
		Turret.getInstance().ifPresent(turret -> turret.setNoteLoadedNoDelay());

		AutoPositionList positions = new AutoPositionList();
		for (int i = 0; i < noteHiearchyChoosers.length && i < shootOrStealChoosers.length; i++) {
			var note = noteHiearchyChoosers[i].getSelected();
			var shootOrSteal = shootOrStealChoosers[i].getSelected();
			if (note != null)
				positions.add(note, shootOrSteal);
		}
		if (positions.isEmpty()) {
			RobotState.getPose().ifPresent((pose) -> {
				var current = pose.getTranslation();
				var x = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)
						? DriveStraightForwardLine.getX()
						: FIELD_DIMENSIONS.LENGTH - DriveStraightForwardLine.getX();
				var driveAcrossLinePosition = new Translation2d(x, current.getY());
				positions.add(new AutoPosition(driveAcrossLinePosition,
						AutoPositionType.Shoot, ShooterType.Speaker)
						.withNotSkippable());
			});
		}
		return Autos.getDynamicAutonomous(positions);
	}

	static LinkedList<PathPlannerPath> autoPaths;
	static int pathIndexOnField;
	static Timer trajectoryOnFieldTimer = new Timer();
	static Trajectory visualizedTrajectory;

	public static void publishAutoTrajectoriesOnField() {
		var auto = guiAutoChooser.detectChange();
		var maxPaths = maxAutoPathsChooser.detectChange();

		if (auto.isPresent() || maxPaths.isPresent()) {
			autoPaths = PathUtils.getAutoPaths(guiAutoChooser.getSelected(), maxAutoPathsChooser.getSelected());
			pathIndexOnField = 0;
			trajectoryOnFieldTimer.restart();
			updateTrajectory();
		}

		if (autoPaths.size() > 0) {

			if (trajectoryOnFieldTimer.advanceIfElapsed(visualizedTrajectory.getTotalTimeSeconds())) {
				pathIndexOnField++;
				if (pathIndexOnField >= autoPaths.size())
					pathIndexOnField = 0;

				updateTrajectory();
			}
		}

		if (autoPaths.size() > 0) {
			RobotState.seedFieldRelativeToInitalPositionIfNoCameraUpdates(autoPaths);
		}
	}

	private static void updateTrajectory() {
		visualizedTrajectory = TelemetryUtils.getWPILIBTrajectory(autoPaths.get(pathIndexOnField));
		CommandSwerveDrivetrain.getInstance().ifPresent(drivetrain -> {
			drivetrain.getField().getObject("traj").setTrajectory(visualizedTrajectory);
		});
	}

	static Optional<PathPlannerPath> displayedPath;

	public static void publishCurrentTrajectoryOnField() {

		var trajectory = TelemetryUtils.getTrajectoryIfChanged(() -> Optional.ofNullable(Autos.getCurrentPath()));

		if (trajectory.isPresent()) {
			CommandSwerveDrivetrain.getInstance().ifPresent(drivetrain -> {
				drivetrain.getField().getObject("traj").setTrajectory(trajectory.get());
			});
		}
	}

}