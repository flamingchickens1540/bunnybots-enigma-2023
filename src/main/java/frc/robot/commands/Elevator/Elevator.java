package frc.robot.commands.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private final CANSparkMax motor = new CANSparkMax(Constants.ElevatorConstants.MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);



     public void setVelocity(double velocity){
        motor.set(velocity);
     }

     public void periodic() {
         SmartDashboard.putBoolean("Elevator/frontLimitSwitch", topLimitHit());
         SmartDashboard.putBoolean("Elevator/backLimitSwitch", bottomLimitHit());
     }

     public boolean topLimitHit(){
         return motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
     }

     public boolean bottomLimitHit(){
         return motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
     }




}
