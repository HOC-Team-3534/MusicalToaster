// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import javax.sound.sampled.Control;

import org.opencv.core.TickMeter;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRequest.ControlIntake;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.ShooterRequest;
import frc.robot.subsystems.turret.ShooterRequest.ControlShooter;
import frc.robot.subsystems.turret.ShooterRequest.Idle;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretRequest;
import frc.robot.subsystems.turret.TurretRequest.ControlTurret;
import frc.robot.subsystems.turret.TurretRequest.IndexFromIntake;
import frc.robot.utils.ControllerUtils;

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

	private final static ControlClimber climb = new ControlClimber().withVoltage(0.3);
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

						var heightDifferenceInches = 60;
						if (distance > Units.feetToMeters(13.0))
							heightDifferenceInches += 2;
						if (distance > 5.25)
							heightDifferenceInches += 1;
						var degrees = Rotation2d
								.fromRadians(Math.atan(Units.inchesToMeters(heightDifferenceInches) / distance))
								.getDegrees();
						return Rotation2d.fromDegrees(degrees);
					}));

	private final static ControlTurret shootStraightForward = new ControlTurret()
			.withTargetAzimuthFunction((turretState) -> Optional.of(new Rotation2d()))
			.withTargetElevationFunction((turretState) -> {
				var degrees = SmartDashboard.getNumber("Tilt", 50);
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

	private static RobotState m_robotState = new RobotState();

	public static class RobotState {
		int grabNoteIndex = -1;
		boolean activelyGrabbing;
		boolean noteLoaded;
		boolean climbing;

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

		SmartDashboard.putNumber("Tilt", 50.0); // TODO Move somewhere else?

		if (Utils.isSimulation()) { // TODO Do we need to set the pose if simulation if we arent really focused on
									// simulation?
			CommandSwerveDrivetrain.getInstance().ifPresent((drivetrain) -> drivetrain
					.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90))));
		}

		createAutonomousChoosers();
	}

	private static void instantiateSubsystems() {
		Constants.ROBOT.getDrivetrain();
		Turret.createInstance();
		Intake.createInstance();
		PhotonVisionCamera.createInstance();
		Climber.createInstance();
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
		for (int i = 0; i < 5; i++) {
			noteHiearchyChoosers[i] = newNoteHiearchyChooser();
			SmartDashboard.putData("Note Hiearchy " + (i + 1), noteHiearchyChoosers[i]);
			shootOrStealChoosers[i] = newShootOrStealChooser();
			SmartDashboard.putData("Shoot|Steal " + (i + 1), shootOrStealChoosers[i]);
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

	}

	public static Command getIntakeAutonomouslyCommand() {
		return Intake.getInstance()
				.map((intake) -> intake.applyRequest(() -> getRobotState().isNoteInRobot() ? stopIntake : runIntake))
				.orElse(Commands.none());
	}

	public enum ShooterType {
		Speaker(aimForSpeaker, shooterSpeaker),
		Steal(aimForSteal, shooterSteal),
		Subwoofer(shootStraightForward, shootSubwoofer);

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
					() -> getRobotState().noteLoaded ? type.turretRequest : indexFromIntake,
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
		return behindAllianceLine && !isTiltForcedFlat();
	}

	public static boolean isTiltForcedFlat() {
		return BTN.TiltFlat.get();
	}

	static final Translation2d DriveStraightForwardLine = FIELD_DIMENSIONS.CENTER_OF_FIELD
			.minus(FIELD_DIMENSIONS.OFFSET_AUTO_CROSS_LINE_FROM_CENTER)
			.plus(new Translation2d(Units.inchesToMeters(25.0), 0));

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public static Command getAutonomousCommand() {
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

	public enum BTN {
		TiltFlat(() -> driverController.getHID().getAButton()),
		Creep(() -> driverController.getHID().getLeftBumper()),
		SubwooferLetItRip(() -> operatorController.getHID().getRightTriggerAxis() > 0.15);

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
		ShootSpeaker(driverController.x(), true),
		Extake(driverController.rightBumper(), false),
		PrepareShootForSubwoofer(operatorController.x(), true),
		Climb(operatorController.start().and(() -> driverController.getHID().getStartButton()), false),

		ResetNoteinRobot(driverController.back(), false),
		// Below are debugging actions

		Characterize(driverController.a().and(() -> EnabledDebugModes.CharacterizeEnabled), false);

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