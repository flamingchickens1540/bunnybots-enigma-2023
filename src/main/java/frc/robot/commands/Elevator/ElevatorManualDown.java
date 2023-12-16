package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class ElevatorManualDown extends CommandBase {
    private final Elevator elevator;


    public ElevatorManualDown(Elevator elevator){
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
//        if ((velocity > 0 && !elevator.topLimitHit()) || (velocity < 0 && !elevator.bottomLimitHit())) {
            elevator.setVelocity(-1);
        //}
    }

    public boolean isFinished() {
        return elevator.bottomLimitHit();
    }

    public void end(boolean interrupted) {
        elevator.setVelocity(0);
    }
}
