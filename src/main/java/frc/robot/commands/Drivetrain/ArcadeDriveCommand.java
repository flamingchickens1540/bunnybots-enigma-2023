package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class ArcadeDriveCommand extends CommandBase {
    private final Drivetrain drivetrain;

    private final CommandXboxController xBoxController;

    private final SlewRateLimiter leftRateLimiter = new SlewRateLimiter(0.75);
    private final SlewRateLimiter rightRateLimiter = new SlewRateLimiter(0.75);

    public ArcadeDriveCommand(Drivetrain drivetrain, CommandXboxController xBoxController) {
        this.drivetrain = drivetrain;
        this.xBoxController = xBoxController;
        addRequirements(drivetrain);
    }

    public void execute() {
        if (xBoxController.getHID().getAButtonPressed()) {drivetrain.invertDrive = !drivetrain.invertDrive;}

        double throttle = MathUtil.applyDeadband(-xBoxController.getLeftY(), Constants.DEADZONE)/2;
        if (drivetrain.getWheelSpeeds().leftMetersPerSecond > 0.5 || drivetrain.getWheelSpeeds().rightMetersPerSecond > 0.5) {throttle =0;}
        double turn =MathUtil.applyDeadband(xBoxController.getRightX(), Constants.DEADZONE)/3;
        double left = leftRateLimiter.calculate(
                MathUtil.clamp(throttle + turn,-1, 1)
        );
        double right  = rightRateLimiter.calculate(
                MathUtil.clamp(throttle - turn, -1, 1)
        );

        drivetrain.setPercent(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setPercent(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}