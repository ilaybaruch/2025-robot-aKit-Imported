package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public ElevatorIO getIO(){
        return elevatorIO;
    }

    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("elevator", elevatorInputs);
        elevatorIO.setPidValues();
    }

}
