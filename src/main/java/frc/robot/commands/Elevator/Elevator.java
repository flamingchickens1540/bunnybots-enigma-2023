package frc.robot.commands.Elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

     private TalonFX motor = new TalonFX(ElevatorConstants.MOTOR_ID);
     
     public Elevator(){
        motor.config_kF(0, ElevatorConstants.ELEVATOR_KF);
        motor.config_kP(0, ElevatorConstants.ELEVATOR_KP);
        motor.config_kI(0, ElevatorConstants.ELEVATOR_KI);
        motor.config_kD(0, ElevatorConstants.ELEVATOR_KD);

        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0));
     }

     public void setPosition(double position){
         //TODO: Write this
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

     public double getPosition(){
         return motor.getSelectedSensorPosition();
     }



}
