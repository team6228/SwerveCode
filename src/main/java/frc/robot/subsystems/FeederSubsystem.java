

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase{

    private final VictorSP feedMotor = new VictorSP(FeederConstants.feederMotor);
    private final VictorSP indexerMotor = new VictorSP(FeederConstants.indexerMotor);

    
    public FeederSubsystem(){}
    

    public void feedShooter(){
        feedMotor.set(-1);
        indexerMotor.set(0.6);
    }

    public void reverseFeed(){
        feedMotor.set(0.5);
        indexerMotor.set(-0.5);
    }

    public void stopMotors(){
        feedMotor.set(0);
        indexerMotor.set(0);
    }
}


