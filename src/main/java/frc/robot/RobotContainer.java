// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import javax.swing.text.html.Option;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drive.FIELD_DIMENSIONS;
import frc.robot.Constants.EnabledDebugModes;
import frc.robot.commands.AutoPosition;
import frc.robot.commands.AutoPosition.AutoPositionType;
import frc.robot.commands.AutoPositionList;
import frc.robot.commands.Autos;
import frc.robot.subsystems.camera.PhotonVisionCamera;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberRequest;
import frc.robot.subsystems.climber.ClimberRequest.ControlClimber;
import frc.robot.subsystems.climber.ClimberRequest.ResetClimber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRequest.ControlIntake;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightModes;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.path.IPathPlanner;
import frc.robot.subsystems.turret.ShooterRequest;
import frc.robot.subsystems.turret.ShooterRequest.ControlShooter;
import frc.robot.subsystems.turret.ShooterRequest.Idle;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretRequest;
import frc.robot.subsystems.turret.TurretRequest.ControlTurret;
import frc.robot.subsystems.turret.TurretRequest.IndexFromIntake;
import frc.robot.subsystems.turret.TurretRequest.JustRoller;
import frc.robot.utils.ControllerUtils;
import frc.robot.utils.SmartDashboardUtils;

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

	private static final CommandXboxController driverController = new CommandXboxController(0);
	private static final CommandXboxController operatorController = new CommandXboxController(1);
	private final static FieldCentric drive = new FieldCentric();

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
			.withTargetAzimuthFunction(turretState -> getPoseRotation()
					.flatMap(robotRotation -> turretState.getVirtualGoalLocationDisplacement()
							.map(displacementToGoal -> displacementToGoal.getAngle().minus(robotRotation))))
			.withTargetElevationFunction(
					(turretState) -> turretState.getVirtualGoalLocationDisplacement().map((displacementToGoal) -> {
						var distance = displacementToGoal.getNorm();
						var a = 2.582;
						var b = -24.096;
						var c = 81.35;
						var degrees = a * Math.pow(distance, 2) + b * distance + c;
						SmartDashboard.putNumber("Distance from Goal", distance);
						return Rotation2d.fromDegrees(degrees);
					}));

	private final static ControlTurret shootStraightForward = new ControlTurret()
			.withTargetAzimuthFunction((turretState) -> Optional.of(new Rotation2d()))
			.withTargetElevationFunction((turretState) -> {
				var degrees = SmartDashboard.getNumber("Tilt", 48);
				return Optional.of(Rotation2d.fromDegrees(degrees));
			})
			.withAllowShootWhenAimedSupplier(() -> BTN.SubwooferLetItRip.get());
	private final static ControlTurret aimForSteal = new ControlTurret()
			.withTargetAzimuthFunction((turretState) -> getPoseRotation()
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
			.withTurnOnRollerSupplier(() -> BTN.ReloadNoteActivate.get());
	private final static ControlTurret prepareForClimbTurret = new ControlTurret()
			.withTargetAzimuthFunction((turretState) -> Optional.of(Rotation2d.fromDegrees(90)))
			.withTargetElevationFunction((turretState) -> Optional.of(new Rotation2d()))
			.withAllowShootWhenAimedSupplier(() -> false);

	private final static Idle shooterOff = new ShooterRequest.Idle();
	private final static ControlShooter shootSubwoofer = new ControlShooter().withVelocity(80);// TODO Find these
	private final static ControlShooter shooterSpeaker = new ControlShooter().withVelocity(80);// TODO Find these
	private final static ControlShooter shooterSteal = new ControlShooter().withVelocity(15);

	private static SendableChooser<Autos.AutoNotes>[] noteHiearchyChoosers = new SendableChooser[5];
	private static SendableChooser<ShooterType>[] shootOrStealChoosers = new SendableChooser[5];

	private static SendableChooser<String> guiAutoChooser = new SendableChooser<>();
	private static SendableChooser<Integer> maxAutoPathsChooser = new SendableChooser<>();

	private static RobotState m_robotState = new RobotState();

	public static class RobotState {
		int grabNoteIndex = -1;
		boolean activelyGrabbing;
		boolean noteLoaded;
		boolean climbing;
		boolean resetingClimber;

		public boolean isActivelyGrabbing() {
			return this.activelyGrabbing;
		}

		public void setActivelyGrabbing(boolean activelyGrabbing) {
			this.activelyGrabbing = activelyGrabbing;
		}

		public boolean isNoteLoaded() {
			return this.noteLoaded;
		}

		public void setNoteLoaded(boolean noteLoaded) {
			this.noteLoaded = noteLoaded;
		}

		public int getGrabNoteIndex() {
			return this.grabNoteIndex;
		}

		public void setGrabNoteIndex(int grabNoteIndex) {
			this.grabNoteIndex = grabNoteIndex;
		}

		public boolean isNoteInRobot() {
			return grabNoteIndex != -1 || noteLoaded;
		}

		public void setClimbing() {
			this.climbing = true;
		}

		public void resetClimbing() {
			this.climbing = false;
		}

		public boolean isClimbing() {
			return this.climbing;
		}

		public void setResetingClimber() {
			this.resetingClimber = true;
		}

		public void resetResetingClimber() {
			this.resetingClimber = false;
		}

		public boolean isResetingClimber() {
			return this.resetingClimber;
		}
	}

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

		SmartDashboard.putNumber("Tilt", 48.0); // TODO Move somewhere else?

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

		Lights.getInstance().ifPresent(lights -> lights.setDefaultCommand(lights.applyLightMode(LightModes.Default)));
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

	public static Optional<Pose2d> getPose() {
		return CommandSwerveDrivetrain.getInstance()
				.map((drivetrain) -> drivetrain.getState().Pose);
	}

	public static Optional<Rotation2d> getPoseRotation() {
		return CommandSwerveDrivetrain.getInstance()
				.map((drivetrain) -> drivetrain.getState().Pose.getRotation());
	}

	public static SendableChooser<Autos.AutoNotes> newNoteHiearchyChooser() {
		SendableChooser<Autos.AutoNotes> noteHiearchy = new SendableChooser<>();

		noteHiearchy.setDefaultOption("None", null);
		for (Autos.AutoNotes note : Autos.AutoNotes.values()) {
			noteHiearchy.addOption(note.name(), note);
		}

		return noteHiearchy;
	}

	public static SendableChooser<ShooterType> newShootOrStealChooser() {
		SendableChooser<ShooterType> shootOrSteal = new SendableChooser<>();

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
						})));

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
		Subwoofer(shootStraightForward, shootSubwoofer),
		ReloadNote(reloadNote, shooterOff);

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
					() -> getRobotState().noteLoaded || type.equals(ShooterType.ReloadNote) ? type.turretRequest
							: indexFromIntake,
					() -> type.shooterRequest);
		}).orElse(Commands.none());
	}

	public static boolean isValidShootPosition() {
		var behindAllianceLine = getPose().map((pose) -> {
			var x = pose.getX();
			var alliance = DriverStation.getAlliance().get();
			return alliance.equals(Alliance.Blue)
					? x < FIELD_DIMENSIONS.CENTER_OF_FIELD.minus(FIELD_DIMENSIONS.OFFSET_ALLIANCE_LINE_FROM_CENTER)
							.getX()
					: x > FIELD_DIMENSIONS.CENTER_OF_FIELD.plus(FIELD_DIMENSIONS.OFFSET_ALLIANCE_LINE_FROM_CENTER)
							.getX();
		}).orElse(true);
		var speeds = CommandSwerveDrivetrain.getInstance().map(drivetrain -> drivetrain.getChassisSpeeds())
				.orElse(new ChassisSpeeds());
		var drivingSlow = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).getNorm() < 0.25;
		return behindAllianceLine && !isTiltForcedFlat() && drivingSlow;
	}

	public static boolean isTiltForcedFlat() {
		return BTN.TiltFlat.get();
	}

	static final Translation2d DriveStraightForwardLine = FIELD_DIMENSIONS.CENTER_OF_FIELD
			.minus(FIELD_DIMENSIONS.OFFSET_AUTO_CROSS_LINE_FROM_CENTER)
			.plus(new Translation2d(Units.inchesToMeters(25.0), 0));

	public static Command getAutonomousCommand() {
		if (Constants.ROBOT.getPathPlanner() instanceof IPathPlanner.PathPlanner2024)
			return getAutonomousCommandDynamic();

		return getAutonomousCommand_FixedFromGUI();
	}

	private static Command getAutonomousCommand_FixedFromGUI() {
		var paths = Autos.getAutoPaths(guiAutoChooser.getSelected(), maxAutoPathsChooser.getSelected());
		return Autos.getGUIAutoCommandNoNamedCommmands(paths);
	}

	private static Command getAutonomousCommandDynamic() {
		Turret.getInstance().ifPresent(turret -> turret.setNoteLoaded());

		AutoPositionList positions = new AutoPositionList();
		for (int i = 0; i < noteHiearchyChoosers.length && i < shootOrStealChoosers.length; i++) {
			var note = noteHiearchyChoosers[i].getSelected();
			var shootOrSteal = shootOrStealChoosers[i].getSelected();
			if (note != null)
				positions.add(note, shootOrSteal);
		}
		if (positions.isEmpty()) {
			getPose().ifPresent((pose) -> {
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

	static String selectedAuto = "";
	static double selectedMaxAutoPaths;
	static LinkedList<PathPlannerPath> autoPaths;
	static int pathIndexOnField;
	static double totalTimeCurrentTrajectory;
	static Timer trajectoryOnFieldTimer = new Timer();
	static Trajectory visualizedTrajectory;

	public static void publishAutoTrajectoriesOnField() {
		if (!selectedAuto.equals(guiAutoChooser.getSelected())
				|| selectedMaxAutoPaths != maxAutoPathsChooser.getSelected()) {
			selectedAuto = guiAutoChooser.getSelected();
			selectedMaxAutoPaths = maxAutoPathsChooser.getSelected();

			autoPaths = Autos.getAutoPaths(guiAutoChooser.getSelected(), maxAutoPathsChooser.getSelected());
			pathIndexOnField = 0;
			trajectoryOnFieldTimer.restart();
			updateTrajectory();
		}

		if (autoPaths.size() > 0) {

			if (trajectoryOnFieldTimer.advanceIfElapsed(totalTimeCurrentTrajectory)) {
				pathIndexOnField++;
				if (pathIndexOnField >= autoPaths.size())
					pathIndexOnField = 0;

				updateTrajectory();
			}
		}
	}

	private static void updateTrajectory() {
		var trajectory = getDisplayTrajectory(autoPaths.get(pathIndexOnField));
		totalTimeCurrentTrajectory = trajectory.getTotalTimeSeconds();
		visualizedTrajectory = getWPILIBTrajectory(trajectory);
		CommandSwerveDrivetrain.getInstance().ifPresent(drivetrain -> {
			drivetrain.getField().getObject("traj").setTrajectory(visualizedTrajectory);
		});
	}

	static Optional<PathPlannerPath> currentDisplayedPath = Optional.empty();
	static boolean wasEmptyLastCheck;

	static Trajectory autoTrajectory;

	public static void publishCurrentTrajectoryOnField() {
		autoTrajectory = null;

		if (currentDisplayedPath.isEmpty() || currentDisplayedPath.get() != Autos.getCurrentPath()) {
			currentDisplayedPath = Optional.ofNullable(Autos.getCurrentPath());

			currentDisplayedPath.ifPresentOrElse(path -> {
				autoTrajectory = getWPILIBTrajectory(getDisplayTrajectory(path));
				wasEmptyLastCheck = false;
			}, () -> {
				if (!wasEmptyLastCheck) {
					autoTrajectory = new Trajectory();
					wasEmptyLastCheck = true;
				}
			});
		}

		if (autoTrajectory != null) {
			CommandSwerveDrivetrain.getInstance().ifPresent(drivetrain -> {
				drivetrain.getField().getObject("traj").setTrajectory(autoTrajectory);
			});
		}
	}

	private static PathPlannerTrajectory getDisplayTrajectory(PathPlannerPath path) {
		var alliancePath = DriverStation.getAlliance()
				.map(alliance -> alliance.equals(Alliance.Red) ? path.flipPath() : path).orElse(path);
		return alliancePath.getTrajectory(new ChassisSpeeds(),
				alliancePath.getPreviewStartingHolonomicPose().getRotation());
	}

	private static Trajectory getWPILIBTrajectory(PathPlannerTrajectory trajectory) {

		List<Trajectory.State> states = trajectory.getStates().stream()
				.map(state -> new Trajectory.State(state.timeSeconds, state.velocityMps, state.accelerationMpsSq,
						SmartDashboardUtils.getPose2dForField2d(state.getTargetHolonomicPose()),
						state.curvatureRadPerMeter))
				.toList();

		return new Trajectory(states);
	}

	public enum BTN {
		TiltFlat(() -> driverController.getHID().getLeftBumper() || operatorController.getHID().getBButton()),
		Creep(() -> driverController.getHID().getLeftTriggerAxis() > 0.15),
		SubwooferLetItRip(() -> operatorController.getHID().getRightBumper()),
		ReloadNoteActivate(() -> SubwooferLetItRip.get()),
		ResetClimber(() -> driverController.getHID().getPOV() == 180 && TiltFlat.get());

		Supplier<Boolean> buttonSupplier;

		BTN(Supplier<Boolean> buttonSupplier) {
			this.buttonSupplier = buttonSupplier;
		}

		public boolean get() {
			return buttonSupplier.get();
		}
	}

	public enum TGR {
		Intake(driverController.rightTrigger(0.15), false),
		ShootSpeaker(operatorController.rightTrigger(0.15), true),
		Extake(driverController.rightBumper(), false),
		PrepareShootForSubwoofer(operatorController.x(), true),
		Climb(operatorController.start().and(() -> driverController.getHID().getStartButton()), false),
		ReloadNote(operatorController.back(), true),

		ResetNoteinRobot(driverController.back(), false),
		// Below are debugging actions

		Characterize(driverController.a().and(() -> EnabledDebugModes.CharacterizeEnabled), false),
		ResetClimber(new Trigger(() -> BTN.ResetClimber.get()).and(() -> !DriverStation.isFMSAttached()), false);

		Trigger trigger;

		TGR(Supplier<Boolean> booleanSupplier, boolean blockedByClimbing) {
			this(new Trigger(() -> booleanSupplier.get()), blockedByClimbing);
		}

		TGR(Trigger trigger, boolean blockedByClimbing) {
			this.trigger = blockedByClimbing ? trigger.and(() -> !getRobotState().isClimbing()) : trigger;
		}

		public Trigger tgr() {
			return trigger.and(() -> !Robot.isAutonomous);
		}
	}

	public enum AXS {
		Drive_ForwardBackward(() -> driverController.getLeftY(), true),
		Drive_LeftRight(() -> driverController.getLeftX(), true),
		Drive_Rotation(() -> driverController.getRightX());

		final Supplier<Double> supplier;
		Optional<SlewRateLimiter> slewRateLimiter = Optional.empty();
		boolean allianceInvert;

		AXS(Supplier<Double> supplier, double rate, boolean allianceInvert) {
			this(supplier, rate);
			this.allianceInvert = allianceInvert;
		}

		AXS(Supplier<Double> supplier, boolean allianceInvert) {
			this(supplier);
			this.allianceInvert = allianceInvert;
		}

		AXS(Supplier<Double> supplier, double rate) {
			this(supplier);
			this.slewRateLimiter = Optional.of(new SlewRateLimiter(rate));
		}

		AXS(Supplier<Double> supplier) {
			this.supplier = () -> ControllerUtils.modifyAxis(-supplier.get(), getDeadband(), getCreepScale())
					* (allianceInvert ? getAllianceInversion() : 1);
		}

		public double getAxis() {
			return applyRateLimit(supplier.get());
		}

		private double applyRateLimit(double value) {
			return slewRateLimiter.map(limiter -> limiter.calculate(value)).orElse(value);
		}

		public static int getAllianceInversion() {
			return DriverStation.getAlliance().map(alliance -> alliance.equals(Alliance.Red) ? -1 : 1).orElse(1);
		}

		public static double getCreepScale() {
			return BTN.Creep.get() ? 0.25 : 1;
		}

		public static double getDeadband() {
			return 0.1;
		}
	}
}