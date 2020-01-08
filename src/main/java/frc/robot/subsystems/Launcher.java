package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    private final CANSparkMax mLaunchMotor;
    
    public Launcher(){
        mLaunchMotor = new CANSparkMax(1, MotorType.kBrushless);
    }
    public void launch(){
       
    }
    public void stop(){

    }
}