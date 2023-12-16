package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

public class Drivetrain extends SubsystemBase{
    
    private final TalonFX rightFront = new TalonFX(Constants.DrivetrainConstants.FRONT_RIGHT_KEY);
    private final TalonFX rightBack = new TalonFX(Constants.DrivetrainConstants.BACK_RIGHT_KEY);
    private final TalonFX leftFront = new TalonFX(Constants.DrivetrainConstants.FRONT_LEFT_KEY);
    private final TalonFX leftBack = new TalonFX(Constants.DrivetrainConstants.BACK_LEFT_KEY);
    private final WPI_Pigeon2 pigeon;
    private final DifferentialDriveOdometry odometry;
    public boolean invertDrive = false;

    public Drivetrain(WPI_Pigeon2 pigeon){
        leftBack.follow(leftFront);
        rightBack.follow(rightFront);
        leftFront.setNeutralMode(NeutralMode.Coast);
        rightFront.setNeutralMode(NeutralMode.Coast);
        rightFront.setInverted(true);
        rightBack.setInverted(true);
        leftFront.setInverted(false);
        leftBack.setInverted(false);

        invertDrive = SmartDashboard.getBoolean("initInvertDrive", false);
        
        this.pigeon = pigeon;
        odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(), 0, 0);
    }

    @Override
    public void periodic() {
        odometry.update(
            pigeon.getRotation2d(), 
            leftFront.getSelectedSensorPosition()/Constants.ENCODER_TICKS_PER_METER,
            rightFront.getSelectedSensorPosition()/Constants.ENCODER_TICKS_PER_METER
        );
    }
    
    public void setPercent(double leftPercent, double rightPercent){
        if (invertDrive) {
            leftFront.set(ControlMode.PercentOutput, -rightPercent);
            rightFront.set(ControlMode.PercentOutput, -leftPercent);
        }
        else {
            leftFront.set(ControlMode.PercentOutput, leftPercent);
            rightFront.set(ControlMode.PercentOutput, rightPercent);
        }
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void setVolts(double leftVolts, double rightVolts){
        if (invertDrive) {
            leftFront.set(ControlMode.PercentOutput, -rightVolts/12);
            rightFront.set(ControlMode.PercentOutput, -leftVolts/12);
        }
        else {
            leftFront.set(ControlMode.PercentOutput, leftVolts/12);
            rightFront.set(ControlMode.PercentOutput, rightVolts/12);
        }
    }
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(
                leftFront.getSelectedSensorVelocity()/Constants.ENCODER_TICKS_PER_METER,
                rightFront.getSelectedSensorVelocity()/Constants.ENCODER_TICKS_PER_METER
        );
    }

    public void resetOdometry(Pose2d pose){
        // navx.reset();
        leftFront.setSelectedSensorPosition(0);
        rightFront.setSelectedSensorPosition(0);
        odometry.resetPosition(pigeon.getRotation2d(), 0, 0, pose);
    }

    public void zeroHeading(){
        pigeon.setYaw(0);
    }

    public void brakeOn(){
        leftFront.setNeutralMode(NeutralMode.Brake);
        rightFront.setNeutralMode(NeutralMode.Brake);
    }
    public void brakeOff(){
        leftFront.setNeutralMode(NeutralMode.Coast);
        rightFront.setNeutralMode(NeutralMode.Coast);
    }
}