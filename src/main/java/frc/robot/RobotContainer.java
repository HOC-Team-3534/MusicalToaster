// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.concurrent.Callable;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
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
import frc.robot.Constants.RobotType;
import frc.robot.commands.AutoPosition;
import frc.robot.commands.AutoPositionList;
import frc.robot.commands.Autos;
import frc.robot.commands.AutoPosition.AutoPositionType;
import frc.robot.generated.TunerConstantsPBOT;
import frc.robot.generated.TunerConstantsTBOT;
import frc.robot.subsystems.camera.PhotonVisionCamera;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRequest.ControlIntake;
import frc.robot.subsystems.swervedrive.FieldCentricWithProperDeadband;
import frc.robot.subsystems.turret.ShooterRequest.ControlShooter;
import frc.robot.subsystems.turret.ShooterRequest.Idle;
import frc.robot.subsystems.turret.ShooterRequest;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretRequest.*;
import swerve.CommandSwerveDrivetrain;

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
	private static boolean isTBOT = Constants.ROBOTTYPE.equals(RobotType.TBOT);
	private static boolean isPBOT = Constants.ROBOTTYPE.equals(RobotType.PBOT);
	public static double kSpeedAt12VoltsMps = isTBOT
			? TunerConstantsTBOT.kSpeedAt12VoltsMps
			: TunerConstantsPBOT.kSpeedAt12VoltsMps;

	private static final CommandXboxController driverController = new CommandXboxController(0);
	private static final CommandXboxController operatorController = new CommandXboxController(1);
	static SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterRotation = new SlewRateLimiter(2.5);
	private final static CommandSwerveDrivetrain drivetrain = isTBOT
			? TunerConstantsTBOT.DriveTrain
			: TunerConstantsPBOT.DriveTrain;
	private final static FieldCentricWithProperDeadband drive = new FieldCentricWithProperDeadband()
			.withDeadband(kSpeedAt12VoltsMps * 0.15).withRotationalDeadband(MaxAngularRate * 0.15)
			.withMaxSpeed(kSpeedAt12VoltsMps).withMaxAngularSpeed(MaxAngularRate)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	// private final static Climber climber = new Climber();
	// private final static ControlClimber climberUp = new
	// ControlClimber().withVoltage(0.5).withClimberReversed(false);
	// private final static ControlClimber climberDown = new
	// ControlClimber().withVoltage(0.5).withClimberReversed(true);

	private final static Turret turret = new Turret(() -> drivetrain.getState(), () -> drivetrain.getChassisSpeeds(),
			() -> TGR.SetNoteLoadedInTurret.bool());

	private final static Intake intake = new Intake(() -> TGR.ResetAllNotePostions.bool());
	private final static ControlIntake runIntake = new ControlIntake().withIntakePercent(1.0);
	private final static ControlIntake runExtake = new ControlIntake().withIntakePercent(-1.0);
	private final static ControlIntake stopIntake = new ControlIntake().withIntakePercent(0.0);

	private final static IndexFromIntake indexFromIntake = new IndexFromIntake()
			.withRollerOutput(-1.0)
			.withTilt(Rotation2d.fromDegrees(40));
	private final static AimForSpeaker aimForSpeaker = new AimForSpeaker()
			.withRollerOutput(0.25)
			.withTiltFunction((distance) -> new Rotation2d())
			.withSwerveDriveState(() -> drivetrain.getState());// TODO Find distance table
	private final static ScoreAmp scoreAmp = new ScoreAmp()
			.withRotation(() -> Rotation2d.fromDegrees(270))
			.withRollerOutput(-0.6)
			.withTilt(Rotation2d.fromDegrees(-30))
			.withSwerveDriveState(() -> drivetrain.getState())
			.withDeployTrigger(() -> TGR.DeployInAmp.bool());// TODO Find the actualy tilt for aiming for amp
	private final static ShootFromSubwoofer shootFromSubwoofer = new ShootFromSubwoofer()
			.withRollerPercent(-1.0)
			.withRotation(new Rotation2d())
			.withTilt(Rotation2d.fromDegrees(48)).withShooterTolerance(5.0);
	private final static AimWithRotation aimForSteal = new AimWithRotation()
			.withRotation(() -> {
				var targetAzimuth = DriverStation.getAlliance().get().equals(Alliance.Blue)
						? Rotation2d.fromDegrees(180)
						: new Rotation2d();
				return targetAzimuth.minus(drivetrain.getState().Pose.getRotation());
			})
			.withRollerOutput(0.25)
			.withTilt(Rotation2d.fromDegrees(10))
			.withSwerveDriveState(() -> drivetrain.getState());// TODO Find the actualy tilt for aiming for amp

	private final static Idle shooterOff = new ShooterRequest.Idle();
	private final static ControlShooter shooterAmp = new ControlShooter().withVelocity(20);// TODO Find these valuess
	private final static ControlShooter shootSubwoofer = new ControlShooter().withVelocity(70);// TODO Find these
	private final static ControlShooter shooterSpeaker = new ControlShooter().withVelocity(80);// TODO Find these
	private final static ControlShooter shooterSteal = new ControlShooter().withVelocity(15);

	private static PhotonVisionCamera photonVision;

	private static SendableChooser<Autos.AutoNotes>[] noteHiearchyChoosers = new SendableChooser[5];
	private static SendableChooser<ShooterType>[] shootOrStealChoosers = new SendableChooser[5];

	private static ReadWriteLock m_stateLock = new ReentrantReadWriteLock();
	private static RobotState m_robotState = new RobotState();

	public static class RobotState {
		public int grabNoteIndex = -1;
		public boolean activelyGrabbing;
		public boolean noteLoaded;
	}

	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public static void initialize() {
		// Configure the trigger bindings
		configureBindings();

		photonVision = new PhotonVisionCamera(() -> drivetrain.getState().Pose,
				(pose, timestamp) -> {
					if (EnabledDebugModes.updatePoseWithVisionEnabled)
						drivetrain.addVisionMeasurement(pose.toPose2d(), timestamp);
				});

		// Set Default Commands for Subsystems
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(
						() -> drive
								.withVelocityX(
										// Drive forward with negative Y (forward)
										-driverController.getLeftY() * kSpeedAt12VoltsMps
												* getCoordinateSystemInversionDriving())
								.withVelocityY(
										// Drive left with negative X (left)
										-driverController.getLeftX() * kSpeedAt12VoltsMps
												* getCoordinateSystemInversionDriving())
								.withRotationalRate(
										// Drive counterclockwise with negative X (left)
										-driverController.getRightX() * MaxAngularRate)
								.withCreepEnabled(TGR.Creep.bool())));

		intake.setDefaultCommand(
				intake.applyRequest(
						() -> isActivelyIndexingFromIntake()
								? runIntake
								: stopIntake));

		turret.setDefaultCommand(
				turret.applyRequest(() -> indexFromIntake, () -> shooterOff));

		// var idle = new TurretRequest.Idle();

		// turret.setDefaultCommand(turret.applyRequest(() -> idle, () -> shooterOff));

		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}

		// Show Status of Subsystems on Dashboard
		for (int i = 0; i < 5; i++) {
			noteHiearchyChoosers[i] = newNoteHiearchyChooser();
			SmartDashboard.putData("Note Hiearchy " + (i + 1), noteHiearchyChoosers[i]);
			shootOrStealChoosers[i] = newShootOrStealChooser();
			SmartDashboard.putData("Shoot|Steal " + (i + 1), shootOrStealChoosers[i]);
		}
	}

	public static SendableChooser<Autos.AutoNotes> newNoteHiearchyChooser() {
		SendableChooser<Autos.AutoNotes> noteHiearchy = new SendableChooser<>();

		noteHiearchy.setDefaultOption("None", null);
		noteHiearchy.addOption("Blue Note 1", Autos.AutoNotes.BlueNote1);
		noteHiearchy.addOption("Blue Note 2", Autos.AutoNotes.BlueNote2);
		noteHiearchy.addOption("Blue Note 3", Autos.AutoNotes.BlueNote3);
		noteHiearchy.addOption("Center Note 4", Autos.AutoNotes.MiddleNote4);
		noteHiearchy.addOption("Center Note 5", Autos.AutoNotes.MiddleNote5);
		noteHiearchy.addOption("Center Note 6", Autos.AutoNotes.MiddleNote6);
		noteHiearchy.addOption("Center Note 7", Autos.AutoNotes.MiddleNote7);
		noteHiearchy.addOption("Center Note 8", Autos.AutoNotes.MiddleNote8);
		noteHiearchy.addOption("Red Note 9", Autos.AutoNotes.RedNote9);
		noteHiearchy.addOption("Red Note 10", Autos.AutoNotes.RedNote10);
		noteHiearchy.addOption("Red Note 11", Autos.AutoNotes.RedNote11);

		return noteHiearchy;
	}

	public static SendableChooser<ShooterType> newShootOrStealChooser() {
		SendableChooser<ShooterType> shootOrSteal = new SendableChooser<>();

		shootOrSteal.setDefaultOption("Shoot", ShooterType.Speaker);
		shootOrSteal.addOption("Steal", ShooterType.Steal);

		return shootOrSteal;
	}

	public static boolean isActivelyIndexingFromIntake() {
		return getRobotState().activelyGrabbing;
	}

	public static RobotState getRobotState() {
		try {
			m_stateLock.readLock().lock();

			return m_robotState;
		} finally {
			m_stateLock.readLock().unlock();
		}
	}

	public static void setActivelyGrabbing(boolean activelyGrabbing) {
		try {
			m_stateLock.writeLock().lock();

			m_robotState.activelyGrabbing = activelyGrabbing;
		} finally {
			m_stateLock.writeLock().unlock();
		}
	}

	public static void setNoteLoaded(boolean noteLoaded) {
		try {
			m_stateLock.writeLock().lock();

			m_robotState.noteLoaded = noteLoaded;
		} finally {
			m_stateLock.writeLock().unlock();
		}
	}

	public static void setGrabNoteIndex(int grabNoteIndex) {
		try {
			m_stateLock.writeLock().lock();

			m_robotState.grabNoteIndex = grabNoteIndex;
		} finally {
			m_stateLock.writeLock().unlock();
		}
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
	 * with an arbitrary predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s
	 * subclasses for {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private static void configureBindings() {
		TGR.Characterize.tgr().whileTrue(drivetrain.characterizeDrive(1.0, 4.0));

		// reset the field-centric heading on left bumper press
		TGR.ResetFieldRelative.tgr().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

		TGR.Intake.tgr().whileTrue(intake.applyRequest(() -> isNoteInRobot() ? stopIntake : runIntake));
		TGR.Extake.tgr().whileTrue(intake.applyRequest(() -> runExtake));

		var justRoller = new JustRoller();

		var shooterAmpPercentage = (new ShooterRequest.ControlShooterPercentage()).withPercentOut(0.2);

		TGR.ShootManually.tgr().whileTrue(turret.applyRequest(() -> justRoller, () -> shooterAmpPercentage));

		// TGR.ShootSpeaker.tgr().whileTrue(getShootCommand(() -> ShooterType.Speaker));
		// TGR.PrepareScoreAmp.tgr().whileTrue(getShootCommand(() -> ShooterType.Amp));
		TGR.PrepareShootForSubwoofer.tgr().whileTrue(getShootCommand(() -> ShooterType.Subwoofer));

		// TGR.ClimbUp.tgr().whileTrue(climber.applyRequest(() -> climberUp));
		// TGR.ClimbDown.tgr().whileTrue(climber.applyRequest(() -> climberDown));

	}

	public static Command getIntakeAutonomouslyCommand() {
		return intake.applyRequest(() -> {
			if (isNoteInRobot())
				return stopIntake;
			return runIntake;
		});
	}

	public static boolean isNoteInRobot() {
		var state = getRobotState();
		return state.grabNoteIndex != -1 || state.noteLoaded;
	}

	public static int getCoordinateSystemInversionDriving() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() && alliance.get().equals(Alliance.Red) ? 1 : -1;
	}

	public enum ShooterType {
		Speaker, Amp, Steal, Subwoofer
	}

	public static Command getShootCommand(Supplier<ShooterType> shooterTypeSupplier) {
		return turret.applyRequest(() -> {
			var type = shooterTypeSupplier.get();
			if (!getRobotState().noteLoaded
					|| (type.equals(ShooterType.Speaker) && !isValidShootPosition()))
				return indexFromIntake;
			switch (type) {
				case Amp:
					return scoreAmp;
				case Speaker:
					return aimForSpeaker;
				case Subwoofer:
					return shootFromSubwoofer.withReadyToShoot(() -> TGR.ShootFromSubwoofer.tgr().getAsBoolean());
				case Steal:
					return aimForSteal;
				default:
					return indexFromIntake;

			}
		}, () -> {
			var type = shooterTypeSupplier.get();
			switch (type) {
				case Amp:
					return shooterAmp;
				case Speaker:
					return shooterSpeaker;
				case Steal:
					return shooterSteal;
				case Subwoofer:
					return shootSubwoofer;
				default:
					return shooterSpeaker;

			}
		});
	}

	public static boolean isValidShootPosition() {
		var x = drivetrain.getState().Pose.getX();
		var alliance = DriverStation.getAlliance().get();
		var behindAllianceLine = alliance.equals(Alliance.Blue)
				? x < FIELD_DIMENSIONS.CENTER_OF_FIELD.minus(FIELD_DIMENSIONS.OFFSET_ALLIANCE_LINE_FROM_CENTER).getX()
				: x > FIELD_DIMENSIONS.CENTER_OF_FIELD.plus(FIELD_DIMENSIONS.OFFSET_ALLIANCE_LINE_FROM_CENTER).getX();
		if (!behindAllianceLine)
			return false;
		return !isTiltForcedFlat();
	}

	public static boolean isTiltForcedFlat() {
		return TGR.ForceTiltFlat.bool();
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
		AutoPositionList positions = new AutoPositionList();
		for (int i = 0; i < noteHiearchyChoosers.length && i < shootOrStealChoosers.length; i++) {
			var note = noteHiearchyChoosers[i].getSelected();
			var shootOrSteal = shootOrStealChoosers[i].getSelected();
			if (note != null)
				positions.add(note, shootOrSteal);
		}
		if (positions.isEmpty()) {
			var current = drivetrain.getState().Pose.getTranslation();
			var x = DriverStation.getAlliance().get().equals(Alliance.Blue) ? DriveStraightForwardLine.getX()
					: FIELD_DIMENSIONS.LENGTH - DriveStraightForwardLine.getX();
			var driveAcrossLinePosition = new Translation2d(x, current.getY());
			positions.add(new AutoPosition(driveAcrossLinePosition,
					AutoPositionType.Shoot, ShooterType.Speaker)
					.withNotSkippable());
		}
		return Autos.getDynamicAutonomous(turret, drivetrain, positions);
	}

	public enum TGR {
		Creep(driverController.leftBumper()),
		ResetFieldRelative(driverController.start()), // TODO should we having this?
		Intake(driverController.rightTrigger(0.15)),
		ShootSpeaker(driverController.x()),
		Extake(driverController.rightBumper()),
		PrepareScoreAmp(driverController.b()),
		DeployInAmp(driverController.y()),
		PrepareShootForSubwoofer(operatorController.x()),
		ShootFromSubwoofer(operatorController.rightTrigger()),
		// ClimbUp(operatorController.a().and(() -> !EnabledDebugModes.testingClimber)),
		// ClimbDown(operatorController.b().and(() ->
		// !EnabledDebugModes.testingClimber)),
		ResetAllNotePostions(operatorController.back()),
		SetNoteLoadedInTurret(operatorController.start()),
		GetRidOfNote(operatorController.a()),
		ForceTiltFlat(operatorController.rightBumper()),

		ShootManually(operatorController.leftTrigger(0.15)),

		// Below are debugging actions

		Characterize(driverController.a().and(() -> EnabledDebugModes.CharacterizeEnabled));

		Trigger trigger;

		TGR(Trigger trigger) {
			this.trigger = trigger;
		}

		public Trigger tgr() {
			return trigger.and(() -> !Robot.isAutonomous);
		}

		public boolean bool() {
			return trigger.getAsBoolean();
		}
	}

	public enum AXS {
		Drive_ForwardBackward(() -> slewRateLimiterX.calculate(-modifyAxis(driverController.getLeftY()))),
		Drive_LeftRight(() -> slewRateLimiterY.calculate(-modifyAxis(driverController.getLeftX()))),
		Drive_Rotation(() -> slewRateLimiterRotation.calculate(-modifyAxis(driverController.getRightX())));

		Callable<Double> callable;

		AXS(Callable<Double> callable) {
			this.callable = callable;
		}

		public double getAxis() {
			try {
				return callable.call().doubleValue();
			} catch (Exception ex) {
				return 0.0;
			}
		}
	}

	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0;
		}
	}

	private static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, 0.1);
		// Square the axis
		value = Math.copySign(value * value, value);
		return value;
	}
}