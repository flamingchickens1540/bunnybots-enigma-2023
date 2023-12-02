// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.DriveCommand;
import frc.robot.commands.Drivetrain.Drivetrain;
import frc.robot.commands.Elevator.Elevator;
import frc.robot.commands.Elevator.ElevatorDown;
import frc.robot.commands.Elevator.ElevatorUp;
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
// run enable compressor
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandGenericHID m_copilotController = new CommandGenericHID(2);
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Drivetrain m_Drivetrain =
      new Drivetrain();
  private final Elevator m_Elevator =
      new Elevator();
  private final PneumaticsControlModule m_pcm =
          new PneumaticsControlModule(25);
  private final Shooter m_shooter =
          new Shooter(m_pcm);
  private final Grabber m_grabber =
          new Grabber(m_pcm);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_Drivetrain.setDefaultCommand(new DriveCommand(m_driverController, m_Drivetrain));
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

    // first button is 0,1,3,2,4
    m_copilotController.button(1).onTrue(new ShooterCommand(m_shooter, 0));
    m_copilotController.button(2).onTrue(new ShooterCommand(m_shooter, 1));
    m_copilotController.button(4).onTrue(new ShooterCommand(m_shooter, 2));
    m_copilotController.button(3).onTrue(new ShooterCommand(m_shooter, 3));
    m_copilotController.button(5).onTrue(new ShooterCommand(m_shooter, 4));
    m_copilotController.axisGreaterThan(4, 0.75).onTrue(new GrabberCommand(m_grabber));
    m_copilotController.povDown().onTrue(new ElevatorDown(m_Elevator));
    m_copilotController.povUp().onTrue(new ElevatorUp(m_Elevator));






  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
