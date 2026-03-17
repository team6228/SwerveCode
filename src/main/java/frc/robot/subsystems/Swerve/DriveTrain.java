package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveContants;
import frc.robot.Constants.targetLock;
import frc.robot.subsystems.QuestNavSubsystem;

public class DriveTrain extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

    private final SwerveModule frontLeft  = new SwerveModule(DriveContants.flDriveCanID, DriveContants.flAngleCanID, DriveContants.flChassisAngularOffset, "fl", 0.01958944275,  3.3008210659);
    private final SwerveModule frontRight = new SwerveModule(DriveContants.frDriveCanID, DriveContants.frAngleCanID, DriveContants.frChassisAngularOffset, "fr", 0.01469208206,  3.29592370986);
    private final SwerveModule backLeft   = new SwerveModule(DriveContants.blDriveCanID, DriveContants.blAngleCanID, DriveContants.blChassisAngularOffset, "bl", 0.01958944275,  3.3008210659);
    private final SwerveModule backRight  = new SwerveModule(DriveContants.brDriveCanID, DriveContants.brAngleCanID, DriveContants.brChassisAngularOffset, "br", 0.01469208206,  3.29102635383);

    // ── Control / Estimation ──────────────────────────────────────────────────
    private final PIDController turnPID = new PIDController(0.01, 0, 0);
    private final SwerveDrivePoseEstimator poseEstimator;

    private QuestNavSubsystem questNav;

    

    // ── Telemetry ─────────────────────────────────────────────────────────────
    private final Field2d m_field = new Field2d();

    private final StructPublisher<Pose2d> posePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("RobotPoseStruct", Pose2d.struct)
            .publish();

    private final StructArrayPublisher<SwerveModuleState> statePublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("SwerveStates", SwerveModuleState.struct)
            .publish();

    // ── Auto pose chooser ─────────────────────────────────────────────────────
    private final SendableChooser<Pose2d> autoPosChooser = new SendableChooser<>();

    // ─────────────────────────────────────────────────────────────────────────
    public DriveTrain() {
        // Register chooser options ONCE (not every periodic tick!)
        autoPosChooser.setDefaultOption("Center",        new Pose2d(3.522, 4.0,  new Rotation2d(0)));
        autoPosChooser.addOption("Left (Red)",           new Pose2d(15.0,  7.0,  Rotation2d.fromDegrees(0)));
        autoPosChooser.addOption("Right (Red)",          new Pose2d(15.0,  1.5,  Rotation2d.fromDegrees(0)));
        SmartDashboard.putData("Auto Start Pose", autoPosChooser);
        SmartDashboard.putData("Update Pose",     new InstantCommand(this::resetPoseToSelected).ignoringDisable(true));
        SmartDashboard.putData("Field",           m_field);

        zeroHeading();

        poseEstimator = new SwerveDrivePoseEstimator(
            DriveContants.kDriveKinematics,
            getRotation2d(),
            getModulePositions(),
            autoPosChooser.getSelected()
        );
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999));

        turnPID.enableContinuousInput(-180, 180);

        configureAutoBuilder();
    }

    // ── PathPlanner ───────────────────────────────────────────────────────────

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                    new PIDConstants(5.65, 0.0, 0.0),
                    new PIDConstants(5.65, 0.0, 0.0)
                ),
                config,
                () -> DriverStation.getAlliance()
                        .map(a -> a == DriverStation.Alliance.Red)
                        .orElse(false),
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner config failed: " + e.getMessage(), true);
        }
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        poseEstimator.update(getRotation2d(), getModulePositions());
        posePublisher.set(getPose());
        m_field.setRobotPose(getPose());

        frontLeft.updateCalibration();
        frontRight.updateCalibration();
        backLeft.updateCalibration();
        backRight.updateCalibration();

        statePublisher.set(new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        });

        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    // ── Drive methods ─────────────────────────────────────────────────────────

    /** Standard field/robot-relative teleop drive. */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed    * DriveContants.maxSpeedMetersPerSecond,
                ySpeed    * DriveContants.maxSpeedMetersPerSecond,
                rotation  * DriveContants.maxAngularSpeed,
                getRotation2d())
            : new ChassisSpeeds(
                xSpeed   * DriveContants.maxSpeedMetersPerSecond,
                ySpeed   * DriveContants.maxSpeedMetersPerSecond,
                rotation * DriveContants.maxAngularSpeed);

        desaturateAndSet(DriveContants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    /** Robot-relative drive used by PathPlanner. */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        desaturateAndSet(DriveContants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    /** Target-tracking drive: auto-rotates to face a field-fixed point. */
    public void driveAtTarget(double xSpeed, double ySpeed) {
    Pose2d currentPose = getPose();
    double dx = targetLock.targetX - currentPose.getX();
    double dy = targetLock.targetY - currentPose.getY();

    double angleToTarget = Math.toDegrees(Math.atan2(dy, dx));

    // Field-relative ySpeed'i robot-local sağ/sol harekete çevir
    double robotHeadingRad = currentPose.getRotation().getRadians();
    double localY = -xSpeed * Math.sin(robotHeadingRad) 
                  +  ySpeed * Math.cos(robotHeadingRad);

    // Sağa giderken (localY > 0) sola bak → negatif offset
    // Sola giderken (localY < 0) sağa bak → pozitif offset
    double lateralOffset = MathUtil.clamp(-localY * 2.0, -2.0, 2.0);

    double angleError = Math.IEEEremainder(getHeading() - (angleToTarget + lateralOffset), 360);

    double rotOutput = (Math.abs(angleError) < 0.5)
        ? 0
        : MathUtil.clamp(turnPID.calculate(angleError, 0), -0.5, 0.5);

    desaturateAndSet(DriveContants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * DriveContants.maxSpeedMetersPerSecond,
            ySpeed * DriveContants.maxSpeedMetersPerSecond,
            rotOutput * DriveContants.maxAngularSpeed,
            getRotation2d()
        )
    ));
}

    /** Lock robot heading to face forward (0°). */
    public void lockFront(double xSpeed, double ySpeed) {
        lockAtAngle(xSpeed, ySpeed, 0);
    }

    /** Lock robot heading to face backward (180°). */
    public void lockBack(double xSpeed, double ySpeed) {
        lockAtAngle(xSpeed, ySpeed, 180);
    }

    /**
     * Drive while holding a fixed field-relative heading.
     * Softens rotation output while translating to avoid lurching.
     */
    private void lockAtAngle(double xSpeed, double ySpeed, double targetDeg) {
        double error      = Math.IEEEremainder(getHeading(), 360);
        double rotOutput  = turnPID.calculate(error, targetDeg);

        boolean moving = Math.abs(xSpeed) > 0.1 || Math.abs(ySpeed) > 0.1;
        if (moving)              rotOutput = MathUtil.clamp(rotOutput, -0.25, 0.25);
        if (Math.abs(error) < 2) rotOutput = 0;

        desaturateAndSet(DriveContants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed * DriveContants.maxSpeedMetersPerSecond,
                ySpeed * DriveContants.maxSpeedMetersPerSecond,
                rotOutput * DriveContants.maxAngularSpeed,
                getRotation2d()
            )
        ));
    }

    /** Arrange wheels in X pattern (defense / resist pushing). */
    public void setX() {
        frontLeft.setDesiredState( new SwerveModuleState(0, Rotation2d.fromDegrees( 45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(  new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState( new SwerveModuleState(0, Rotation2d.fromDegrees( 45)));
    }

    // ── Module helpers ────────────────────────────────────────────────────────

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        desaturateAndSet(desiredStates);
    }

    /** Desaturate wheel speeds then push states to all four modules. */
    private void desaturateAndSet(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveContants.maxSpeedMetersPerSecond);
        frontLeft.setDesiredState( states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(  states[2]);
        backRight.setDesiredState( states[3]);
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    // ── Pose / Odometry ───────────────────────────────────────────────────────

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void resetPoseToSelected() {
        Pose2d pose = autoPosChooser.getSelected();
        if (pose != null) {
            resetOdometry(pose);
            System.out.println("Pose reset to: " + pose);
        }
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }

    // ── Gyro ──────────────────────────────────────────────────────────────────

    public Rotation2d getRotation2d() {
        //double angle = navx.getAngle();
        //return Rotation2d.fromDegrees(DriveContants.gyroReversed ? angle : -angle);
        // Eğer QuestNav bağlıysa ve takip ediyorsa ondan al
        if (questNav != null && questNav.isTracking()) {
            return questNav.getPose2d().getRotation();
        }
        
        // YEDEK: QuestNav yoksa veya takip koptuysa NavX kullanmaya devam et
        // (Veya istersen burayı tamamen NavX'siz yapabilirsin ama güvenlik için kalması iyidir)
        double angle = navx.getAngle();
        return Rotation2d.fromDegrees(DriveContants.gyroReversed ? angle : -angle);
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public void zeroHeading() {
        navx.reset();
        if (questNav != null) {
            questNav.zeroPose(); // QuestNav'ı sıfırla
        }
    }

    // ── Misc ──────────────────────────────────────────────────────────────────

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveContants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(), frontRight.getState(),
            backLeft.getState(),  backRight.getState()
        );
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void setQuestNav(QuestNavSubsystem questNav) {
        this.questNav = questNav;
    }
}