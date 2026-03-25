package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.targetLock;
import frc.robot.subsystems.Swerve.DriveTrain;

public class ShooterTestSubsystem extends SubsystemBase {

    // -------------------------------------------------------------------------
    // Donanım
    // -------------------------------------------------------------------------
    private final AnalogInput hoodPotInput = new AnalogInput(ShooterConstants.hoodPotPort);
    private final VictorSP    hoodMotor    = new VictorSP(ShooterConstants.hoodMotorPWM);

    private final SparkMax masterNeo = new SparkMax(ShooterConstants.masterNeoCanID, MotorType.kBrushless);
    private final SparkMax follower1 = new SparkMax(ShooterConstants.follower1NeoCanID, MotorType.kBrushless);
    private final SparkMax follower2 = new SparkMax(ShooterConstants.follower2NeoCanID, MotorType.kBrushless);
    private final SparkClosedLoopController shooterPID;
    private final InterpolatingDoubleTreeMap velocityTable = new InterpolatingDoubleTreeMap();

    //private final FeederSubsystem feeder = new FeederSubsystem();

    // -------------------------------------------------------------------------
    // Durum
    // -------------------------------------------------------------------------
    /** Aktif hedef RPM — hem statik hem dinamik mod tarafından güncellenir. */
    private double  targetRPM        = 0;
    private boolean isShooterRunning = false;

    /**
     * true iken periodic(), hood PID setpoint'ini ve targetRPM'i EZMEZ.
     * setDynamicTarget() bu bayrağı açar, disablePID() kapatır.
     */
    private boolean dynamicMode = false;

    // -------------------------------------------------------------------------
    // Swerve
    // -------------------------------------------------------------------------
    private final DriveTrain m_driveTrain;

    // -------------------------------------------------------------------------
    // Interpolasyon Tablosu — Mesafe(m) → Hood Açısı(°)
    // -------------------------------------------------------------------------
    private final InterpolatingDoubleTreeMap hoodMap     = new InterpolatingDoubleTreeMap();
    private final Translation2d kHubLocation;

    // -------------------------------------------------------------------------
    // Potansiyometre Sabitleri
    // -------------------------------------------------------------------------
    private static final double kMinPotValue     = 4.0;
    private static final double kMaxPotValue     = 4007.0;
    private static final double kPotTotalDegrees = 270.0;

    // -------------------------------------------------------------------------
    // Hood Açısı Offset
    // -------------------------------------------------------------------------
    private static final double kHoodAngleOffset = 30.0;

    // -------------------------------------------------------------------------
    // PID
    // -------------------------------------------------------------------------
    private final PIDController hoodPID = new PIDController(0.1, 0.0, 0.001);

    private static final double kAngleTolerance = 1.5;

    private boolean pidEnabled = false;

    // =========================================================================
    // Constructor
    // =========================================================================
    public ShooterTestSubsystem(DriveTrain driveTrain) {
        this.m_driveTrain = driveTrain;
        this.shooterPID   = masterNeo.getClosedLoopController();
        if(driveTrain.allianceSelector() == "RED"){
            kHubLocation = new Translation2d(targetLock.targetXRed, targetLock.targetYRed);
        }
        else{
            kHubLocation = new Translation2d(targetLock.targetXBlue, targetLock.targetYBlue);
        }
        hoodMotor.setInverted(true);
        hoodPID.setTolerance(kAngleTolerance);

        setupMap();
        configureShooterMotors();
    }

    private void configureShooterMotors() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop
            .p(0.00042)
            .i(0)
            .d(0.00004)
            .outputRange(-1, 1);

        config.inverted(true);

        config.closedLoop.feedForward.kV(0.000202);
        config.closedLoopRampRate(0.1);

        masterNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(11);
        follower1.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower2.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // =========================================================================
    // Interpolasyon Tablosu
    // =========================================================================
    private void setupMap() {
        hoodMap.put(1.0,  3.0);
        hoodMap.put(2.0,  6.0);
        hoodMap.put(3.0, 15.70);
        hoodMap.put(4.0, 18.5);
        hoodMap.put(5.0, 30.0);
        hoodMap.put(6.0, 30.0);
        hoodMap.put(7.0, 30.0);

        velocityTable.put(1.0, 2700.0);
        velocityTable.put(2.0, 2700.0);
        velocityTable.put(3.0, 3000.0);
        velocityTable.put(4.0, 3200.0);
        velocityTable.put(5.0, 3500.0);
        velocityTable.put(6.0, 3700.0);
        velocityTable.put(7.0, 4500.0);
    }

    // =========================================================================
    // Sensör Okuma
    // =========================================================================

    public double getHoodCurrentAngle() {
        double raw   = (double) hoodPotInput.getValue();
        double angle = (raw - kMinPotValue) * kPotTotalDegrees / (kMaxPotValue - kMinPotValue);
        angle -= kHoodAngleOffset;
        return angle;
    }

    public double getDistanceToHub() {
        Pose2d currentPose = m_driveTrain.getPose();
        return currentPose.getTranslation().getDistance(kHubLocation);
    }

    public double getTargetAngle() {
        return hoodMap.get(getDistanceToHub());
    }

    // =========================================================================
    // PID Kontrol
    // =========================================================================

    /**
     * Statik mod PID — mesafeye göre sabit setpoint ile hood'u hedefe götürür.
     * dynamicMode'u kapatır.
     */
    public void enablePID() {
        dynamicMode = false;
        hoodPID.setSetpoint(getTargetAngle());
        pidEnabled = true;
    }

    /**
     * Hood motorunu durdurur, PID'i sıfırlar, dinamik modu da kapatır.
     * Her iki modun (statik + dinamik) çıkışında çağrılmalıdır.
     */
    public void disablePID() {
        pidEnabled  = false;
        dynamicMode = false;
        hoodMotor.set(0.0);
        hoodPID.reset();
    }

    public boolean isAtTarget() {
        return hoodPID.atSetpoint();
    }

    public void setHoodSpeed() {
        disablePID();
        hoodMotor.set(0.3);
    }

    // =========================================================================
    // Periodic
    // =========================================================================
    @Override
    public void periodic() {
        double currentAngle = getHoodCurrentAngle();
        double distance     = getDistanceToHub();

        // dynamicMode aktifken periodic(), hood PID setpoint'ini ve targetRPM'i
        // statik değerlerle EZMEz. Dinamik setpoint her execute() döngüsünde
        // setDynamicTarget() tarafından zaten güncelleniyor.
        double targetAngle;
        if (!dynamicMode) {
            targetAngle = getTargetAngle();
            targetRPM   = velocityTable.get(distance);

            if (pidEnabled) {
                hoodPID.setSetpoint(targetAngle);
            }
        } else {
            // Dinamik moddayken SmartDashboard için mevcut setpoint'i göster
            targetAngle = hoodPID.getSetpoint();
        }

        double error = targetAngle - currentAngle;

        if (pidEnabled) {
            double output = hoodPID.calculate(currentAngle);
            output = Math.max(-0.5, Math.min(0.5, output));
            hoodMotor.set(output);
        }

        double  currentRPM     = masterNeo.getEncoder().getVelocity();
        boolean isShooterReady = Math.abs(currentRPM - targetRPM) < 50.0;
        boolean isHoodReady    = Math.abs(currentAngle - targetAngle) < 1.5;
        boolean isShootReady   = isShooterReady && isHoodReady;

        SmartDashboard.putNumber ("Hood/Mevcut Aci",        currentAngle);
        SmartDashboard.putNumber ("Hood/Hedef Aci",         targetAngle);
        SmartDashboard.putNumber ("Hood/Hub Mesafesi (m)",  distance);
        SmartDashboard.putNumber ("Hood/Hata (°)",          error);
        SmartDashboard.putNumber ("Hood/Motor Cikisi",      hoodMotor.get());
        SmartDashboard.putNumber ("Hood/Offset (°)",        kHoodAngleOffset);
        SmartDashboard.putBoolean("Hood/Hedefe Ulasti",     isAtTarget());
        SmartDashboard.putBoolean("Hood/PID Aktif",         pidEnabled);
        SmartDashboard.putBoolean("Hood/Dinamik Mod",       dynamicMode);
        SmartDashboard.putBoolean("Shooter/isShootReady",   isShootReady);
        SmartDashboard.putBoolean("Shooter/Hiz Tamam",      isShooterReady);
        SmartDashboard.putNumber ("Shooter/Hiz",            currentRPM);
        SmartDashboard.putNumber ("Shooter/Hedef Hiz",      targetRPM);
    }

    // =========================================================================
    // Shooter Kontrol — Statik Mod
    // =========================================================================

    /** Statik mesafeye göre hesaplanan targetRPM ile shooter'ı çalıştırır. */
    public void shoot() {
        isShooterRunning = true;  // ← EKLENDİ: bayrak güncelleniyor
        shooterPID.setSetpoint(targetRPM, SparkMax.ControlType.kVelocity);
    }
 
    public void stopShooter() {
        isShooterRunning = false;
        masterNeo.set(0);         // ← duty cycle moduna geç, PID bırak
    }

    /**
     * Shooter ve hood'un hedefe ulaşıp ulaşmadığını kontrol eder.
     * Dinamik modda targetRPM dinamik değeri tutar, hoodPID.getSetpoint()
     * dinamik açıyı tutar — her iki mod için de doğru sonuç döner.
     */
    public boolean isReady() {
        double  currentRPM     = masterNeo.getEncoder().getVelocity();
        double  currentAngle   = getHoodCurrentAngle();
        boolean isShooterReady = Math.abs(currentRPM - targetRPM) < 50.0;
        boolean isHoodReady    = Math.abs(currentAngle - hoodPID.getSetpoint()) < 1.5;
        return isShooterReady && isHoodReady;
    }

    public void toggleShooter() {
        isShooterRunning = !isShooterRunning;
 
        if (isShooterRunning) {
            shooterPID.setSetpoint(targetRPM, SparkMax.ControlType.kVelocity);
        } else {
            masterNeo.set(0);     // ← DÜZELTİLDİ: setSetpoint(0) yerine set(0)
        }
    }

    // =========================================================================
    // Dinamik Atış Parametresi Hesabı
    // driveAtTarget() mantığıyla paralel: konum + hıza göre hood açısı ve RPM.
    // Hiçbir donanıma veya mevcut duruma dokunmaz — sadece hesaplama yapar.
    // =========================================================================

    /**
     * Robot'un mevcut konumunu VE hızını dikkate alarak dinamik hood açısı
     * ve hedef RPM hesaplar.
     *
     * <p>Statik interpolasyon tablolarından farkı: robot hedefe dik yönde
     * (lateral) hareket ediyorsa top havadayken ileri gideceğinden effektif
     * mesafe kompanzasyonu uygulanır. driveAtTarget() içindeki lead-offset
     * mantığıyla aynı fiziği kullanır.
     *
     * @return Hesaplanan {@link ShotParameters} (targetHoodAngle, targetRPM).
     */
    public ShotParameters getDynamicShotParameters() {

        // ------------------------------------------------------------------
        // 1. Mevcut konum ve statik mesafe
        // ------------------------------------------------------------------
        Pose2d currentPose = m_driveTrain.getPose();

        double dx             = kHubLocation.getX() - currentPose.getX();
        double dy             = kHubLocation.getY() - currentPose.getY();
        double staticDistance = Math.hypot(dx, dy);

        // ------------------------------------------------------------------
        // 2. Robot hızını field-relative'e çevir
        // ------------------------------------------------------------------
        ChassisSpeeds robotSpeeds   = m_driveTrain.getRobotRelativeSpeeds();
        Rotation2d    robotRotation = currentPose.getRotation();

        double fieldVx = robotSpeeds.vxMetersPerSecond * robotRotation.getCos()
                       - robotSpeeds.vyMetersPerSecond * robotRotation.getSin();
        double fieldVy = robotSpeeds.vxMetersPerSecond * robotRotation.getSin()
                       + robotSpeeds.vyMetersPerSecond * robotRotation.getCos();

        double totalSpeed = Math.hypot(fieldVx, fieldVy);

        // ------------------------------------------------------------------
        // 3. Hedefe yönelik birim vektör (field-relative)
        // ------------------------------------------------------------------
        double hubDirX = (staticDistance > 0.01) ? (dx / staticDistance) : 1.0;
        double hubDirY = (staticDistance > 0.01) ? (dy / staticDistance) : 0.0;

        // Hedefe dik (lateral) hız bileşeni — cross product (2D)
        double lateralVelocity = fieldVx * hubDirY - fieldVy * hubDirX;

        // Hedefe doğru (radial) hız bileşeni — dot product
        // Pozitif = robot hub'a yaklaşıyor, negatif = hub'dan uzaklaşıyor
        // (hubDir vektörü robottan hub'a işaret eder)
        double radialVelocity = fieldVx * hubDirX + fieldVy * hubDirY;

        // ------------------------------------------------------------------
        // 4. Effektif mesafe hesabı (lead kompanzasyonu)
        // ------------------------------------------------------------------
        final double kFlightTime    = 0.5;   // saniye — mekanik takıma göre kalibre et
        final double kLateralWeight = 0.3;   // lateral etkinin ağırlığı (0–1)
        final double kRadialWeight  = 1.0;  // radial (yaklaşma/uzaklaşma) etkisi
        final double kMinDistance   = 1.5;   // metre — tablonun alt sınırı
        final double kMaxDistance   = 7.0;   // metre — tablonun üst sınırı
        final double kMaxSpeedMPS   = 10.0;  // DriveContants.maxSpeedMetersPerSecond ile eşleştir

        // Lateral mesafe offseti: robot yana kayarken top daha geniş yay çizer
        double lateralOffset = Math.abs(lateralVelocity) * kFlightTime * kLateralWeight;

        // Radial offset: hub'a yaklaşıyorsa mesafe azalır (offset negatif),
        // uzaklaşıyorsa mesafe artar (offset pozitif).
        double radialOffset = -radialVelocity * kFlightTime * kRadialWeight;

        // Hız büyüklüğüne göre blend: çok yavaşken offset neredeyse sıfır olsun
        double speedBlend = MathUtil.clamp(totalSpeed / kMaxSpeedMPS, 0.0, 1.0);

        double effectiveDistance = staticDistance + (lateralOffset + radialOffset) * speedBlend;
        effectiveDistance = MathUtil.clamp(effectiveDistance, kMinDistance, kMaxDistance);

        // ------------------------------------------------------------------
        // 5. Interpolasyon tablolarından değerleri çek
        // ------------------------------------------------------------------
        double dynamicHoodAngle = hoodMap.get(effectiveDistance);
        double dynamicRPM       = velocityTable.get(effectiveDistance);

        // ------------------------------------------------------------------
        // 6. SmartDashboard — debug
        // ------------------------------------------------------------------
        SmartDashboard.putNumber("DynamicShot/StatikMesafe (m)",     staticDistance);
        SmartDashboard.putNumber("DynamicShot/EffektifMesafe (m)",   effectiveDistance);
        SmartDashboard.putNumber("DynamicShot/LateralHiz (m/s)",     lateralVelocity);
        SmartDashboard.putNumber("DynamicShot/RadialHiz (m/s)",      radialVelocity);
        SmartDashboard.putNumber("DynamicShot/HizBlend",             speedBlend);
        SmartDashboard.putNumber("DynamicShot/HedefHoodAcisi (deg)", dynamicHoodAngle);
        SmartDashboard.putNumber("DynamicShot/HedefRPM",             dynamicRPM);

        /*if(currentRPMDynamic == dynamicRPM){
            feeder.feedShooter();
        }
        */
        return new ShotParameters(dynamicHoodAngle, dynamicRPM);
        
    }

    /**
     * getDynamicShotParameters() sonucunu doğrudan hood PID'ine ve shooter
     * closed-loop kontrolcüsüne uygular.
     *
     * <p>dynamicMode bayrağını açarak periodic()'in statik değerlerle üzerine
     * yazmasını engeller. targetRPM alanını da günceller ki isReady() her iki
     * modda doğru çalışsın. Bumper bırakıldığında stopShooter() + disablePID()
     * çağrılmalıdır — disablePID() dynamicMode'u da kapatır.
     *
     * @param params getDynamicShotParameters() tarafından döndürülen değerler
     */
    public void setDynamicTarget(ShotParameters params) {
        dynamicMode = true;
        pidEnabled  = true;

        // targetRPM'i güncelle ki isReady() doğru çalışsın
        targetRPM = params.targetRPM;

        hoodPID.setSetpoint(params.targetHoodAngle);
        shooterPID.setSetpoint(params.targetRPM, SparkMax.ControlType.kVelocity);
    }

    // =========================================================================
    // ShotParameters — getDynamicShotParameters() dönüş tipi
    // Değişmez (immutable) veri sınıfı; motor/sensor durumuna etki etmez.
    // =========================================================================

    public static final class ShotParameters {
        /** Hedef hood açısı (derece). */
        public final double targetHoodAngle;
        /** Hedef shooter hızı (RPM). */
        public final double targetRPM;

        public ShotParameters(double targetHoodAngle, double targetRPM) {
            this.targetHoodAngle = targetHoodAngle;
            this.targetRPM       = targetRPM;
        }
    }
}



