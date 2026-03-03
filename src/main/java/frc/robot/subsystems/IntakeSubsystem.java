package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    private final VictorSP intakeMotor = new VictorSP(5);
    public IntakeSubsystem(){

    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Intake Ready", intakeSolenoid.get() == DoubleSolenoid.Value.kForward);
    }

    public void openIntake(){
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);

    }

    public void closeIntake(){
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void intakeToggle(){
        intakeSolenoid.toggle();
    }

    public void runIntake(double speed){
        intakeMotor.set(speed);
    }
}
