package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class DriveCommand extends CommandBase {
    public final Drivetrain drivetrain;
    private final XboxController controller;
    public double leftInput;
    public double rightInput;
    public boolean invertDrive = false;

    public DriveCommand(CommandXboxController controller, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.controller = controller.getHID();

        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.set(0, 0);
    }

    @Override
    public void execute() {
        if (controller.getAButtonPressed()) {
            invertDrive = !invertDrive;
        }

        leftInput = controller.getLeftY();
        leftInput = Math.abs(leftInput) > Constants.DEADZONE ? leftInput : 0;
        leftInput += controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
        leftInput *= 0.5;

        rightInput = controller.getRightY();
        rightInput = Math.abs(rightInput) > Constants.DEADZONE ? rightInput : 0;
        rightInput += controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
        rightInput *= 0.5;

        if (invertDrive) {
            drivetrain.set(-leftInput, -rightInput);
        } else {
            drivetrain.set(rightInput, leftInput);
        }
    }
}
