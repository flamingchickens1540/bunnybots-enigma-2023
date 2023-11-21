package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class ElevatorUp extends CommandBase{
    private final Elevator elevator;


    
    public ElevatorUp(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setVelocity(Constants.ElevatorConstants.ELEVATOR_ASCENSION_VELOCITY);
    }

    @Override
    public boolean isFinished() {
        return elevator.topLimitHit();
    }
    @Override
    public void end(boolean interrupted) {
        elevator.setVelocity(0);
    }
}
