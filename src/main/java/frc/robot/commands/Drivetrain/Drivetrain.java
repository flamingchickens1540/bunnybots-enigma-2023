package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase{
    public TalonFX frontLeft = new TalonFX(Constants.DrivetrainConstants.FRONT_LEFT_KEY);
    public TalonFX backLeft = new TalonFX(Constants.DrivetrainConstants.BACK_LEFT_KEY);
    public TalonFX frontRight = new TalonFX(Constants.DrivetrainConstants.FRONT_RIGHT_KEY);
    public TalonFX backRight = new TalonFX(Constants.DrivetrainConstants.BACK_RIGHT_KEY);

    public Drivetrain() {
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        frontLeft.setNeutralMode(NeutralMode.Brake);
        backLeft.setNeutralMode(NeutralMode.Brake);
        frontRight.setNeutralMode(NeutralMode.Brake);
        backRight.setNeutralMode(NeutralMode.Brake);

        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        frontLeft.setInverted(false);
        backLeft.setInverted(false);
    }

    public void set(double percentRight, double percentLeft) {
        frontLeft.set(ControlMode.PercentOutput, percentLeft);
        frontRight.set(ControlMode.PercentOutput, percentRight);
    }
    
}
