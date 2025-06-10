package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    //private final ElevatorIOAutoLogged elevatorInputs = new ElevatorIOAutoLogged();

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public ElevatorIO getIO(){
        return elevatorIO;
    }

    // public void periodic() {
    //     elevatorIO.updateInputs(elevatorInputs);
    //     Logger.processInputs("elevator", elevatorInputs);
    // }

}
