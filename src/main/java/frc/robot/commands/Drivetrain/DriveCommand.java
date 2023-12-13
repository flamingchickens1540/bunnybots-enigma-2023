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
        drivetrain.setPercent(0, 0);
    }

    @Override
    public void execute() {
        if (controller.getAButtonPressed()) {invertDrive = !invertDrive;}

        leftInput = controller.getLeftY();
        leftInput = Math.abs(leftInput) > Constants.DEADZONE ? 0.5*leftInput : 0;
        leftInput = invertDrive ? -leftInput : leftInput;

        rightInput = controller.getRightY();
        rightInput = Math.abs(rightInput) > Constants.DEADZONE ? 0.5*rightInput : 0;
        rightInput = invertDrive ? -rightInput : rightInput;

        drivetrain.setPercent(leftInput, rightInput);
    }
}
