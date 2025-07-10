package frc.robot.Commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Subsystems.Elevator.Elevator;

public class ElevatorCommnads extends Command {
    Elevator elevator;

    public ElevatorCommnads(Elevator elevator) {
        this.elevator = elevator;
    }

    public Command runElevator() {
        return Commands.runEnd(() -> elevator.getIO().setSpeed(0.15), () -> elevator.getIO().stopElevator(), elevator);
    }

    public Command reverseElevator() {
        return Commands.runEnd(() -> elevator.getIO().setSpeed(-0.05), () -> elevator.getIO().stopElevator(), elevator);
    }

    public Command runElevatorPID(double goal) {
        return Commands.runEnd(() -> elevator.getIO().runPID(goal), () -> elevator.getIO().setSpeed(0), elevator);
    }

    public Command runElevatorPIDFF(double goal, double velocity) {
        return new FunctionalCommand(() -> {
        }, () -> elevator.getIO().runPIDWithFF(goal, velocity),
                interrupted -> elevator.getIO().runFF(velocity), () -> elevator.getIO().getPos() == goal,
                elevator);

    }

    public Command runFF(double goal, double velocity) {
        return Commands.runEnd(() -> elevator.getIO().setFeedForward(goal, velocity),
                () -> elevator.getIO().stopElevator(),
                elevator);
    }

    public Command stopElevator() {
        return Commands.run(() -> elevator.getIO().stopElevator(), elevator);
    }
}
