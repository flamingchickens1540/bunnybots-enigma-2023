package frc.robot.commands.Elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

     private final TalonFX motor = new TalonFX(Constants.ElevatorConstants.MOTOR_ID);
     
     public Elevator(){
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0));
     }



     public void setVelocity(double velocity){
        motor.set(ControlMode.PercentOutput, velocity);
     }

     public boolean topLimitHit(){
         return motor.isFwdLimitSwitchClosed()==1;
     }

     public boolean bottomLimitHit(){
         return motor.isRevLimitSwitchClosed()==1;
     }




}
