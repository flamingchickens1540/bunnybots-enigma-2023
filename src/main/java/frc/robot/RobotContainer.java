// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsControlModule;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.ArcadeDriveCommand;
import frc.robot.commands.Drivetrain.AutoCommand;
import frc.robot.commands.Drivetrain.DriveCommand;
import frc.robot.commands.Drivetrain.Drivetrain;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Grabber.Grabber;
import frc.robot.commands.Grabber.GrabberCommand;
import frc.robot.commands.Shooter.Shooter;
import frc.robot.commands.Shooter.ShooterCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandGuitarHeroController m_copilotController =
          new CommandGuitarHeroController(OperatorConstants.kCopilotControllerPort);
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Elevator m_Elevator =
          new Elevator();
  private final PneumaticsControlModule m_pcm =
          new PneumaticsControlModule(OperatorConstants.kPCM_ID);
  private final Shooter m_shooter =
          new Shooter(m_pcm);
  private final Grabber m_grabber =
          new Grabber(m_pcm);

  private final Drivetrain m_Drivetrain =
          new Drivetrain(new WPI_Pigeon2(Constants.DrivetrainConstants.PIGEON_CAN_ID));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    if (SmartDashboard.getBoolean("isArcadeDrive", true)) {
      m_Drivetrain.setDefaultCommand(new ArcadeDriveCommand(m_Drivetrain, m_driverController));
    }
    else {
      m_Drivetrain.setDefaultCommand(new DriveCommand(m_driverController, m_Drivetrain));
    }
//    m_Elevator.setDefaultCommand(new ElevatorManual(m_Elevator, m_copilotController));
    m_pcm.enableCompressorDigital();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

      m_copilotController.button1().onTrue(new ShooterCommand(m_shooter, 1));
      m_copilotController.button2().onTrue(new ShooterCommand(m_shooter, 2));
      m_copilotController.button3().onTrue(new ShooterCommand(m_shooter, 3));
      m_copilotController.button4().onTrue(new ShooterCommand(m_shooter, 4));
      m_copilotController.button5().onTrue(new ShooterCommand(m_shooter, 5));
    
    m_copilotController.triggerAxis().onTrue(new InstantCommand(() -> m_grabber.set(!m_grabber.get())));
    m_copilotController.povDown().whileTrue(new ElevatorDown(m_Elevator));
    m_copilotController.povUp().whileTrue(new ElevatorUp(m_Elevator));
    m_copilotController.button7().onTrue(m_Elevator.stopElevator());
    m_copilotController.button8().onTrue(m_Elevator.stopElevator());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String autoChoice) {
    return new AutoCommand(m_Drivetrain, autoChoice, m_Elevator, m_grabber, m_shooter);
  }
}
