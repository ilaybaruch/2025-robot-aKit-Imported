package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Elevator.*;

public class ElevatorCommnads extends Command {
    public Command runElevator(Elevator elevator){
        return Commands.runEnd(()->elevator.getIO().setSpeed(0),()-> elevator.getIO().stopElevator(), elevator);
    }
}
