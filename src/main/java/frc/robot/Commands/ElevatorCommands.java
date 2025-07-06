package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Subsystems.Elevator.Elevator;

public class ElevatorCommands extends Command {

    Elevator elevator;

    public ElevatorCommands(Elevator elevator) {
        this.elevator = elevator;
    }

    public Command runPIDWithFF(double goal, double velocity) {
        return new FunctionalCommand(() -> {
        }, () -> elevator.getIO().setPIDWithFF(goal, velocity),
                interrupted -> elevator.getIO().setFF(velocity, goal), () -> elevator.getIO().getPos() == goal,
                elevator);
    }

    public Command runFF(double velocity, double goal) {
        return Commands.runEnd(() -> elevator.getIO().setFF(velocity, goal), () -> elevator.getIO().stopMotor(),
                elevator);
    }

    public Command setSpeed(double precentage) {
        return Commands.runEnd(() -> elevator.getIO().setSpeed(precentage), () -> elevator.getIO().stopMotor(),
                elevator);
    }
}
