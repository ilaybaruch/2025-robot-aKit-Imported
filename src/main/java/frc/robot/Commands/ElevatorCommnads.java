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
        return Commands.runEnd(() -> elevator.getIO().setSpeed(0.15), () -> elevator.getIO().stopElevator(), elevator)
                .withName("move up manual");
    }

    public Command reverseElevator() {
        return Commands.runEnd(() -> elevator.getIO().setSpeed(-0.05), () -> elevator.getIO().stopElevator(), elevator)
                .withName("move down manual");
    }

    public Command runElevatorPID(double goal) {
        return Commands.runEnd(() -> elevator.getIO().runPID(goal), () -> elevator.getIO().stopElevator(), elevator)
                .withName("PID");
    }

    public Command runElevatorPIDFF(double goal) {
        return new FunctionalCommand(elevator.getIO()::resetPID, () -> elevator.getIO().runPIDWithFF(goal, 0),
                interrupted -> elevator.getIO().runFF(0), () -> elevator.getIO().atGoal(),
                elevator).withName("PID With FF");

    }

    public Command elevatorDown() {
        return new FunctionalCommand(elevator.getIO()::resetPID, () -> elevator.getIO().runPIDWithFF(0, 0),
                interrupted -> elevator.getIO().runFF(0), () -> elevator.getIO().atGoal(),
                elevator)
                .andThen(
                        elevator.startEnd(() -> elevator.getIO().setSpeed(-0.05), () -> elevator.getIO().stopElevator())
                                .until(() -> elevator.getIO().isPressed()))
                .withName("elevator down");
    }

    public Command runFF(double goal, double velocity) {
        return Commands.runEnd(() -> elevator.getIO().setFeedForward(goal, velocity),
                () -> elevator.getIO().stopElevator(),
                elevator).withName("FF");
    }

    public Command stopElevator() {
        return Commands.run(() -> elevator.getIO().stopElevator(), elevator).withName("stop");
    }
}
