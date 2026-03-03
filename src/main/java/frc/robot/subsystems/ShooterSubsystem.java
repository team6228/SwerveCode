

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Swerve.DriveTrain;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;



public class ShooterSubsystem extends SubsystemBase {
    
    private final SparkMax masterNeo = new SparkMax(ShooterConstants.masterNeoCanID, MotorType.kBrushless);
    private final SparkMax follower1 = new SparkMax(ShooterConstants.follower1NeoCanID, MotorType.kBrushless);
    private final SparkMax follower2 = new SparkMax(ShooterConstants.follower2NeoCanID, MotorType.kBrushless);
    private final SparkClosedLoopController shooterPID;

    private final VictorSP hoodMotor = new VictorSP(ShooterConstants.hoodMotorPort); 
    private final AnalogPotentiometer hoodPot = new AnalogPotentiometer(ShooterConstants.hoodPotPort, ShooterConstants.hoodPotRange, ShooterConstants.hoodPotOffset); // Analog 0
    
    private final DriveTrain m_driveTrain; 
    private final PIDController hoodPID = new PIDController(0.05, 0, 0); 
    private final Translation2d targetLocation = new Translation2d(12.924406, 4.034536);

    private final InterpolatingDoubleTreeMap velocityTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();

    private double targetAngle = 0;
    private double targetRPM = 0;
    

    public ShooterSubsystem(DriveTrain driveTrain) {
        this.m_driveTrain = driveTrain;
        this.shooterPID = masterNeo.getClosedLoopController();

        configureShooterMotors();
        setupInterpolation();
        
        hoodPID.setTolerance(1.0);
    }

    private void configureShooterMotors() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.closedLoop
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1);
            
        config.closedLoopRampRate(0.3);
        
        config.closedLoop.feedForward.kV(0.00017);
        
        masterNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(10); // Master ID'yi takip et
        follower1.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower2.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setupInterpolation() {
        velocityTable.put(1.5, 2500.0);
        velocityTable.put(3.0, 3800.0);
        velocityTable.put(5.0, 5200.0);

        hoodTable.put(1.5, 15.0);
        hoodTable.put(3.0, 32.0);
        hoodTable.put(5.0, 55.0);
    }

    @Override
    public void periodic() {
        // 1. Odomatriden Mevcut Konumu ve Mesafeyi Al
        Pose2d currentPose = m_driveTrain.getPose();
        double distance = currentPose.getTranslation().getDistance(targetLocation) - 5.23;

        // 2. Mesafeye Göre Interpolasyon Tablosundan Hedef Değerleri Çek
        targetRPM = velocityTable.get(distance);
        targetAngle = hoodTable.get(distance);

        // 3. Mevcut Durum Verilerini Oku
        double currentRPM = masterNeo.getEncoder().getVelocity();
        double currentPotValue = hoodPot.get();

        // 4. "Hazır mı?" Kontrolleri (Toleranslar: 50 RPM ve 1.5 Derece)
        boolean isShooterReady = Math.abs(currentRPM - targetRPM) < 50.0;
        boolean isHoodReady = Math.abs(currentPotValue - targetAngle) < 1.5;
        
        // Hem hız hem açı istenen aralıktaysa atışa hazırız
        boolean isShootReady = isShooterReady && isHoodReady;

        // 5. Hood'u Sürekli Hedef Açıya Sür (VictorSP + PID)
        double hoodOutput = hoodPID.calculate(currentPotValue, targetAngle);
        
        // Güvenlik Limitleri: Silecek motorunun fiziksel sınırları (Örn: 5-65 derece)
        if (currentPotValue < 5 && hoodOutput < 0) {
            hoodMotor.set(0);
        } else if (currentPotValue > 65 && hoodOutput > 0) {
            hoodMotor.set(0);
        } else {
            hoodMotor.set(hoodOutput);
        }

        // 6. Dashboard Verilerini Güncelle
        SmartDashboard.putNumber("Shooter/Uzaklik (m)", distance);
        SmartDashboard.putNumber("Shooter/Hedef RPM", targetRPM);
        SmartDashboard.putNumber("Shooter/Mevcut RPM", currentRPM);
        SmartDashboard.putNumber("Shooter/Hedef Hood Acisi", targetAngle);
        SmartDashboard.putNumber("Shooter/Mevcut Pot", currentPotValue);
        
        // Hazır Bilgisi (Işık gibi yanar)
        SmartDashboard.putBoolean("Shooter/isShootReady", isShootReady);
        SmartDashboard.putBoolean("Shooter/Hiz Tamam", isShooterReady);
        SmartDashboard.putBoolean("Shooter/Hood Tamam", isHoodReady);
    }

    public void shoot() {
        shooterPID.setSetpoint(targetRPM, SparkMax.ControlType.kVelocity);
    }

    public void stopShooter() {
        shooterPID.setSetpoint(0, SparkMax.ControlType.kVelocity);
    }

    // ShooterSubsystem.java içine ekle:
    public boolean isReady() {
        double currentRPM = masterNeo.getEncoder().getVelocity();
        double currentPotValue = hoodPot.get();
        
        boolean isShooterReady = Math.abs(currentRPM - targetRPM) < 50.0;
        boolean isHoodReady = Math.abs(currentPotValue - targetAngle) < 1.5;
        
        return isShooterReady && isHoodReady;
    }
}

