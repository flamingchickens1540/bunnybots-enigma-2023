package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase{
    public TalonFX frontLeft = new TalonFX(Constants.DrivetrainConstants.FRONT_LEFT_KEY);
    public TalonFX backLeft = new TalonFX(Constants.DrivetrainConstants.BACK_LEFT_KEY);
    public TalonFX frontRight = new TalonFX(Constants.DrivetrainConstants.FRONT_RIGHT_KEY);
    public TalonFX backRight = new TalonFX(Constants.DrivetrainConstants.BACK_RIGHT_KEY);

    private final DifferentialDriveOdometry odometry;
    private final Pigeon2 pigeon;


    public Drivetrain(Pigeon2 pigeon) {
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

        this.pigeon = pigeon;
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(pigeon.getYaw()), frontLeft.getSelectedSensorPosition(), frontLeft.getSelectedSensorPosition());

        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getWheelSpeeds, // Current ChassisSpeeds supplier
            this::setVelocity, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(pigeon.getYaw()),
            frontLeft.getSelectedSensorPosition()/Constants.DrivetrainConstants.ENCODER_TICKS_PER_METER,
            frontRight.getSelectedSensorPosition()/Constants.DrivetrainConstants.ENCODER_TICKS_PER_METER
        );
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public ChassisSpeeds getWheelSpeeds(){
        return RamseteConfig.getChassisSpeeds(new DifferentialDriveWheelSpeeds(
                frontLeft.getSelectedSensorVelocity()*10/Constants.DrivetrainConstants.ENCODER_TICKS_PER_METER,
                frontRight.getSelectedSensorVelocity()*10/Constants.DrivetrainConstants.ENCODER_TICKS_PER_METER
        )); 
    }

    public void resetOdometry(Pose2d pose){
        frontLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);
        zeroHeading();
        odometry.resetPosition(new Rotation2d(-Math.PI/2), frontLeft.getSelectedSensorPosition(), frontLeft.getSelectedSensorPosition(), pose);
        // new Rotation2d()
    }

    public double getYaw(){
        return pigeon.getYaw();
    }

    public void zeroHeading(){
        pigeon.setYaw(0);
    }    

    public TalonFXSensorCollection getLeftSensors(){
        return frontLeft.getSensorCollection();
    }
    public TalonFXSensorCollection getRightSensors(){
        return frontRight.getSensorCollection();
    }

    public void set(double percentRight, double percentLeft) {
        frontLeft.set(ControlMode.PercentOutput, percentLeft);
        frontRight.set(ControlMode.PercentOutput, percentRight);
    }

    public void setVelocity(ChassisSpeeds speeds) {
        double[] wheelSpeeds = RamseteConfig.getWheelSpeeds(speeds);
        frontLeft.set(ControlMode.Velocity, wheelSpeeds[0]);
        frontRight.set(ControlMode.Velocity, wheelSpeeds[1]);
    }
    
}
