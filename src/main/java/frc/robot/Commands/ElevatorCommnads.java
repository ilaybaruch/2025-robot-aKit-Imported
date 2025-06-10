package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Elevator.*;

public class ElevatorCommnads extends Command {
    public Command runElevator(Elevator elevator){
        return Commands.runEnd(()->elevator.getIO().setSpeed(0),()-> elevator.getIO().stopElevator(), elevator);
    }

    public Command runElevatorPID(Elevator elevator, double goal){
        return Commands.runEnd(()->elevator.getIO().runPID(goal), ()->elevator.getIO().stopElevator(),elevator);
    }

    public Command runElevatorPIDFF(Elevator elevator, double goal, double velocity){
        return Commands.runEnd(()->elevator.getIO().runPIDWithFF(goal, velocity),()-> elevator.getIO().stopElevator(),elevator);
    }

    public Command stopElevator(Elevator elevator){
        return Commands.run(()->elevator.getIO().stopElevator(), elevator);
    }
}
