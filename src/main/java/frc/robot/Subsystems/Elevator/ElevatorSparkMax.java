
package frc.robot.Subsystems.Elevator;

import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;
import frc.robot.Subsystems.Elevator.ElevatorTuning;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSparkMax implements ElevatorIO {

    private final ElevatorTuning pidConstants;
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput limitSwitch;
    ProfiledPIDController pidController;
    private ElevatorFeedforward feedforward;

    public ElevatorSparkMax() {
        limitSwitch = new DigitalInput(1);
        pidConstants = new ElevatorTuning();
        motor = new SparkMax(MOTOR_ID,MotorType.kBrushless );
        encoder =  motor.getEncoder();
        feedforward = new ElevatorFeedforward(Ks, Kg, Kv, Ka);
        pidController = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
                
                
                SparkBaseConfig config = new SparkMaxConfig();
                
                config.idleMode(IdleMode.kBrake).inverted(INVERTED)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        config.encoder.positionConversionFactor(1.0)
                .velocityConversionFactor(1.0 / 60.0);
                
                motor.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        encoder.setPosition(0);

        pidController.setTolerance(TOLERANCE);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.velocity = encoder.getVelocity();
        inputs.voltage = (motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.limitSwitch = limitSwitch.get();
    }

    @Override
    public double getFeedForward(double velocity) {
        double voltage = pidController.calculate(velocity);
        if (encoder.getPosition() >= UP_POS) {
            voltage += pidConstants.upperKg.get();
        }
        return voltage;
    }

    @Override
    public void setSpeed(double precentage) {
        motor.set(precentage);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stopElevator() {
        motor.stopMotor();
    }

    @Override
    public void setFeedForward(double velocity) {
        motor.setVoltage(getFeedForward(velocity));
    }

    @Override
    public void runPID(double goal) {
        motor.set(pidController.calculate(encoder.getPosition(),goal));
    }

    @Override
    public void runPIDWithFF(double goal) {
        motor.setVoltage(getFeedForward(0));
    }

    @Override
    public void setPidValues(){
        pidController.setP(pidConstants.getKp());
        pidController.setI(pidConstants.getKi());
        pidController.setD(pidConstants.getKd());
        pidController.setConstraints(new TrapezoidProfile.Constraints(pidConstants.getMaxVelocity(), pidConstants.getMaxAcceleration()));
        feedforward = new ElevatorFeedforward(pidConstants.getKs(), pidConstants.getKg(), pidConstants.getKv());

    }
}
