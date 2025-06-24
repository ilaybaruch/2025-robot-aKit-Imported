package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;


public class ElevatorSimBuiltIn implements ElevatorIO {
    ElevatorSim elevator;
    ProfiledPIDController pidController;
    private final ElevatorTuning pidConstants;

    public ElevatorSimBuiltIn(){
        elevator = new ElevatorSim(DCMotor.getNEO(1), GEARING, ELEVATOR_MASS, DRUM_RAD, ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT, true, ELEVATOR_START_HEIGHT, null);
        pidController = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        pidConstants = new ElevatorTuning();
    }

    @Override
    public double getFeedForward(double velocity) {
        double voltage = pidController.calculate(velocity);
        if (elevator.getPositionMeters()*METER_TO_POS >= UP_POS) {
            voltage += pidConstants.upperKg.get();
        }
        return voltage;
    }

    @Override
    public void setSpeed(double speed ) {
        elevator.setInput(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        elevator.setInputVoltage(voltage);
    }

    @Override
    public void stopElevator() {
        elevator.setInput(0);
    }

    @Override
    public void setFeedForward(double velocity) {
        elevator.setInputVoltage(getFeedForward(velocity));
    }

    @Override
    public void runPID(double goal) {
        elevator.setInputVoltage(pidController.calculate(elevator.getPositionMeters()*METER_TO_POS,goal));
    }

    @Override
    public void runPIDWithFF(double goal) {
        elevator.setInputVoltage(getFeedForward(0));
    }
    

    
}
