// package frc.robot.Subsystems.Elevator;

// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

// import org.littletonrobotics.junction.Logger;


// public class ElevatorIOSim implements ElevatorIO {
//     DCMotorSim elevator;
//     ProfiledPIDController pidController;
//     private final ElevatorTuning pidConstants;

//     public ElevatorIOSim(){
//         elevator = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 1, 21), DCMotor.getNEO(1),null/*for noise not necessary for now  */ );
//         pidController = new ProfiledPIDController(Kp, Ki, Kd,
//                 new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
//         pidConstants = new ElevatorTuning();
//     }

//     // @Override
//     // public double getFeedForward(double velocity) {
//     //     double voltage = pidController.calculate(velocity);
//     //     if (elevator.getAngularPositionRotations() >= UP_POS) {
//     //         voltage += pidConstants.upperKg.get();
//     //     }
//     //     return voltage;
//     // }

//     @Override
//     public void setSpeed(double speed ) {
//         elevator.setInput(speed);
//     }

//     @Override
//     public void setVoltage(double voltage) {
//         elevator.setInputVoltage(voltage);
//     }

//     @Override
//     public void stopElevator() {
//         elevator.setInput(0);
//     }

//     @Override
//     public void setFeedForward(double velocity) {
//         elevator.setInputVoltage(getFeedForward(velocity));
//     }

//     @Override
//     public void runPID(double goal) {
//         elevator.setInputVoltage(pidController.calculate(elevator.getAngularPositionRotations(),goal));
//     }

//     @Override
//     public void runPIDWithFF(double goal) {
//         elevator.setInputVoltage(getFeedForward(0));
//     }

//     @Override
//     public void updateInputs(ElevatorIOInputs inputs) {
//         elevator.update(UPDATE_SEC);
//         inputs.Position = elevator.getAngularPositionRotations();
//         inputs.velocity = elevator.getAngularVelocityRPM();
//         inputs.voltage = elevator.getOutput(1);
//         Logger.recordOutput("elevator position", elevator.getAngularPositionRotations());
//     }

    

// }
