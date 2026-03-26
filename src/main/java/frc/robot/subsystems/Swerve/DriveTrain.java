package frc.robot.subsystems.Swerve;

import java.util.Optional;

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

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();


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

    private final SendableChooser<String> colorChooser = new  SendableChooser<>();

    // ─────────────────────────────────────────────────────────────────────────
    public DriveTrain() {
        autoPosChooser.setDefaultOption("Center (Blue)",    new Pose2d(3.5, 4.0, new Rotation2d(0)));
        autoPosChooser.addOption("Left (Blue)",       new Pose2d(3.5,  5.1, Rotation2d.fromDegrees(0)));
        autoPosChooser.addOption("Right (Blue)",      new Pose2d(3.5,  2.9, Rotation2d.fromDegrees(0)));

        autoPosChooser.addOption("Center (Red)",       new Pose2d(13.018,  4.041, Rotation2d.fromDegrees(180)));
        autoPosChooser.addOption("Right (Red)",      new Pose2d(13.024,  2.9, Rotation2d.fromDegrees(180)));
        autoPosChooser.addOption("Left (Red)",       new Pose2d(13.013,  5.084, Rotation2d.fromDegrees(180)));
        SmartDashboard.putData("Auto Start Pose", autoPosChooser);
        SmartDashboard.putData("Update Pose",     new InstantCommand(this::resetPoseToSelected).ignoringDisable(true));
        SmartDashboard.putData("Field",           m_field);

        colorChooser.setDefaultOption("RED", getName());
        colorChooser.addOption("BLUE", getName());
        SmartDashboard.putData("Color Chooser",colorChooser);

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

        String allianceStr = alliance.isPresent() ? alliance.get().toString() : "UNKNOWN";

        SmartDashboard.putString("Alliance", allianceStr);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Color", allianceSelector());
        
    }

    // ── Drive methods ─────────────────────────────────────────────────────────

    /** Standard field/robot-relative teleop drive. */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed   * DriveContants.maxSpeedMetersPerSecond,
                ySpeed   * DriveContants.maxSpeedMetersPerSecond,
                rotation * DriveContants.maxAngularSpeed,
                getRotation2d())
            : new ChassisSpeeds(
                xSpeed   * DriveContants.maxSpeedMetersPerSecond,
                ySpeed   * DriveContants.maxSpeedMetersPerSecond,
                rotation * DriveContants.maxAngularSpeed);

        desaturateAndSet(DriveContants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public Pose2d getResetPose(){
        return autoPosChooser.getSelected();
    }

    /** Robot-relative drive used by PathPlanner. */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        desaturateAndSet(DriveContants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public String allianceSelector() {
        String x = colorChooser.getSelected();
        return x;
    }

    public void driveAtTarget(double xSpeed, double ySpeed) {
        // 1. Hedef Koordinatlar

        double targetX;
        double targetY;

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetX = 11.898000;
            targetY = 4.013000;
        } else {
            targetX = 4.625594;
            targetY = 4.034536;
        }

        Pose2d currentPose = getPose();

        // 2. Hedef vektörü (field-relative)
        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();
        double distanceToTarget = Math.hypot(dx, dy);

        // 3. Robot frame'ine çevir (field -> robot)
        Rotation2d robotRot = currentPose.getRotation();

        double localX =  dx * robotRot.getCos() + dy * robotRot.getSin();
        double localY = -dx * robotRot.getSin() + dy * robotRot.getCos();

        // -------------------------------------------------
        // 🔥 GERÇEK ROBOT HIZI (odometriden)
        // -------------------------------------------------

        ChassisSpeeds robotSpeeds = getRobotRelativeSpeeds(); // mevcut gerçek hız
        // Field-relative'e çevir
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotSpeeds, getRotation2d()
        );

        double realVx = fieldSpeeds.vxMetersPerSecond; // field X hızı
        double realVy = fieldSpeeds.vyMetersPerSecond; // field Y hızı
        double realSpeed = Math.hypot(realVx, realVy);  // toplam hız (m/s)

        // Robot frame'inde lateral hız (hedefe dik bileşen)
        double realLocalVx =  realVx * robotRot.getCos() + realVy * robotRot.getSin();
        double realLocalVy = -realVx * robotRot.getSin() + realVy * robotRot.getCos();

        // -------------------------------------------------
        // 🔥 HAREKET ALGILAMA (gerçek hıza göre)
        // -------------------------------------------------

        double movementThreshold = 0.08; // m/s — titreşim eşiği
        boolean isMoving = realSpeed > movementThreshold;

        // -------------------------------------------------
        // 🔥 AIM OFFSET HESABI
        // -------------------------------------------------

        double angleOffset = 0.0;

        if (isMoving && localX > 0.0) { // Hedef önümüzdeyse ve hareket varsa

            // (A) Hıza bağlı lead offset
            // Hedefe dik (lateral) hareket ediyorsak öne bakmalıyız
            double kVelocityAim = 3.0; // derece / (m/s)
            double velocityOffset =
                MathUtil.clamp(realLocalVy * kVelocityAim, -6.0, 6.0);

            // (B) Uzaklığa bağlı pozisyon offseti
            // Hedefe yakınken offset azalsın (zaten doğrultudayız)
            // Uzaktayken biraz daha agresif
            double kPosAim = 1.2; // derece / metre
            double distanceFactor = MathUtil.clamp(distanceToTarget / 4.0, 0.0, 1.0);
            double positionalOffset =
                MathUtil.clamp(localY * kPosAim * distanceFactor, -5.0, 5.0);

            // (C) Hız büyüklüğüne göre blend — yavaşken offset küçülsün
            // 0 m/s → 0.0, maxSpeed → 1.0
            double speedFactor =
                MathUtil.clamp(realSpeed / DriveContants.maxSpeedMetersPerSecond, 0.0, 1.0);

            angleOffset = (velocityOffset + positionalOffset) * speedFactor;
        }
        // isMoving == false → angleOffset = 0.0, direkt hedefe bak

        // 4. Hedef açısı + ofset
        double angleToTarget =
            Math.toDegrees(Math.atan2(dy, dx)) + angleOffset;

        // 5. PID ile dönüş
        double angleError =
            Math.IEEEremainder(getHeading() - angleToTarget, 360);

        double rotationOutput = turnPID.calculate(angleError, 0);
        rotationOutput = MathUtil.clamp(rotationOutput, -0.5, 0.5);

        // Deadband (titreşim kesici)
        if (Math.abs(angleError) < 0.5) {
            rotationOutput = 0;
        }

        // 6. Şasi hızları
        double xVel = xSpeed * DriveContants.maxSpeedMetersPerSecond;
        double yVel = ySpeed * DriveContants.maxSpeedMetersPerSecond;
        double rVel = rotationOutput * DriveContants.maxAngularSpeed;

        // 7. Field-relative sürüş
        ChassisSpeeds chassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVel,
                yVel,
                rVel,
                getRotation2d()
            );

        // 8. Modüllere gönder
        var states =
            DriveContants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
            states,
            DriveContants.maxSpeedMetersPerSecond
        );

        setModuleStates(states);
    }

    /** Lock robot heading to face forward (0°). */
    public void lockFront(double xSpeed, double ySpeed) {
        lockAtAngle(xSpeed, ySpeed, 0);
    }

    /** Lock robot heading to face backward (180°). */
    public void lockBack(double xSpeed, double ySpeed) {
        lockAtAngle(xSpeed, ySpeed, 180);
    }

    /** Drive while holding a fixed field-relative heading. */
    private void lockAtAngle(double xSpeed, double ySpeed, double targetDeg) {
        double error     = Math.IEEEremainder(getHeading(), 360);
        double rotOutput = turnPID.calculate(error, targetDeg);

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
        if (pose == null) return;
        
        poseEstimator.resetPosition(
            getRotation2d(), 
            getModulePositions(), 
            pose
        );
        // ** zeroHeading() 
        // Önce odometriyi sıfırla
        resetOdometry(pose);

        // Sonra QuestNav'ı aynı pose'a sıfırla
        // (QuestNav artık offset değil, gerçek setPose() kullanıyor)
        if (questNav != null) {
            questNav.zeroPose(pose);
        }

        System.out.println("Pose reset to: " + pose);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }

    // ── Gyro ──────────────────────────────────────────────────────────────────

    public Rotation2d getRotation2d() {
        /*if (questNav != null && questNav.isTracking()) {
            return questNav.getPose2d().getRotation();
        }*/
        double angle = navx.getAngle();
        return Rotation2d.fromDegrees(DriveContants.gyroReversed ? angle : -angle);
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public void zeroHeading() {
        navx.reset();

        // questNav burada henüz null olabilir (constructor'da çağrılır)
        // getPose() ile mevcut poz korunur
        if (questNav != null) {
            questNav.zeroPose(getPose());
        }
    }

    // ── Misc ──────────────────────────────────────────────────────────────────

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveContants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),  frontRight.getState(),
            backLeft.getState(),   backRight.getState()
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

    public double getFieldRelativeVX() {
        ChassisSpeeds robotSpeeds = getRobotRelativeSpeeds();
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, getRotation2d()).vxMetersPerSecond;
    }

    /** Robotun field-relative Y eksenindeki hızını döndürür (m/s). */
    public double getFieldRelativeVY() {
        ChassisSpeeds robotSpeeds = getRobotRelativeSpeeds();
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, getRotation2d()).vyMetersPerSecond;
    }
}