package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class ElevatorDown extends CommandBase{
    private final Elevator elevator;


    
    public ElevatorDown(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setVelocity(Constants.ElevatorConstants.ELEVATOR_DESCENSION_VELOCITY);
    }

    @Override
    public boolean isFinished() {
        return elevator.bottomLimitHit();
    }
    
    @Override
    public void end(boolean interrupted) {
        elevator.setVelocity(0);
    }
}

