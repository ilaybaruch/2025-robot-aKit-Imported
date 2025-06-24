package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;


public class ElevatorSimBuiltIn implements ElevatorIO {
    ElevatorSim elevator;
    ProfiledPIDController pidController;
    private final ElevatorTuning pidConstants;
    Pose3d pose3d;

    public ElevatorSimBuiltIn(){
        elevator = new ElevatorSim(DCMotor.getNEO(1), GEARING, ELEVATOR_MASS, DRUM_RAD, ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT, true, ELEVATOR_START_HEIGHT, null);
        pidController = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        pidConstants = new ElevatorTuning();
        pose3d = new Pose3d(0, 0, elevator.getPositionMeters(), new Rotation3d());
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
        elevator.setInputVoltage(0);
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
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevator.update(UPDATE_SEC);
        inputs.Position = elevator.getPositionMeters();
        inputs.velocity = elevator.getVelocityMetersPerSecond();
        inputs.voltage = elevator.getOutput(1);
        inputs.pose = pose3d;
        Logger.recordOutput("elevator position", elevator.getPositionMeters());
        Logger.recordOutput("elevator pose", pose3d);
    }
    
}
