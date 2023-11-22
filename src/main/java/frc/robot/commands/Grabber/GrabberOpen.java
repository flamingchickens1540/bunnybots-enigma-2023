package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberOpen extends CommandBase {
    private final Grabber grabber;
    public GrabberOpen(Grabber grabber) {
        this.grabber = grabber;
        addRequirements(grabber);
    }
    public boolean isFinished() {
        return grabber.get();
    }
}
