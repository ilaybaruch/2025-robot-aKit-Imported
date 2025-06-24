// package frc.robot.Subsystems.Elevator;

// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;


// public class ElevatorIOSim implements ElevatorIO {
//     DCMotorSim elevator;
//     ProfiledPIDController pidController;
//     private final ElevatorTuning pidConstants;

//     public ElevatorIOSim(){
//         elevator = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0, 0), DCMotor.getNEO(1),null/*for noise not necessary for now  */ );
//         pidController = new ProfiledPIDController(Kp, Ki, Kd,
//                 new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
//         pidConstants = new ElevatorTuning();
//     }

//     @Override
//     public double getFeedForward(double velocity) {
//         double voltage = pidController.calculate(velocity);
//         if (elevator.getAngularPositionRad() >= UP_POS) {
//             voltage += pidConstants.upperKg.get();
//         }
//         return voltage;
//     }

//     @Override
//     public void setSpeed(double radPerSec ) {
//         elevator.setAngularVelocity(radPerSec);
//     }

//     @Override
//     public void setVoltage(double voltage) {
//         elevator.setInputVoltage(voltage);
//     }

//     @Override
//     public void stopElevator() {
//         elevator.setAngularVelocity(0);
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
    

// }
