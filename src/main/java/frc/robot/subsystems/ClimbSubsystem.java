package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private final DoubleSolenoid climbSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    
    private boolean shooterReady = SmartDashboard.getBoolean("Shooter/Hiz Tamam", false);

    public ClimbSubsystem(){
        
        compressor.disable();//compresor.enableDigital() olarak değiştir çalışmazsa!!!

    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Climb/IsCompEnable", compressor.isEnabled());
        SmartDashboard.putBoolean("Climb/IsPressureReady", compressor.getPressureSwitchValue());
        SmartDashboard.putBoolean("Climb/ClimbReady", climbSolenoid.get() == DoubleSolenoid.Value.kForward);


        //Çalışmazsa sil ve 20. satırı düzelt
        if(shooterReady){
            compressor.enableDigital();
        } else{
            compressor.disable();
        }
    }

    public void climbForward(){
        climbSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void climbReverse(){
        climbSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void climbToggle() {
        
        if (climbSolenoid.get() == DoubleSolenoid.Value.kForward) {
            climbSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else {
            climbSolenoid.set(DoubleSolenoid.Value.kForward);
        }
    }


}