package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class DriveCommand extends CommandBase {
    public final Drivetrain drivetrain;
    private final CommandXboxController controller;
    public double leftInput;
    public double rightInput;
    public boolean invertDrive = false;

    public DriveCommand(CommandXboxController controller, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.controller = controller;

        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.set(0, 0);
    }

    @Override
    public void execute() {
        if (controller.getHID().getAButtonPressed()) {
            invertDrive = !invertDrive;
        }

        leftInput = controller.getLeftY();
        leftInput = Math.abs(leftInput) > Constants.DEADZONE ? 0.5*leftInput : 0;
        leftInput = invertDrive ? -leftInput : leftInput;

        rightInput = controller.getRightY();
        rightInput = Math.abs(rightInput) > Constants.DEADZONE ? 0.5*rightInput : 0;
        rightInput = invertDrive ? -rightInput : leftInput;

        drivetrain.set(rightInput, leftInput);
    }
}
