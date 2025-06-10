
package frc.robot.Subsystems.Elevator;

import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

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

    private final SparkMax motor = new SparkMax(0,MotorType.kBrushless );
    private final RelativeEncoder encoder = motor.getEncoder();
    private final DigitalInput limitSwitch = new DigitalInput(0);
    ProfiledPIDController pidController;
    private ElevatorFeedforward feedforward;

    public ElevatorSparkMax() {

        feedforward = new ElevatorFeedforward(Ks, Kg, Kv, Ka);
        pidController = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
                
                SparkBaseConfig config = new SparkMaxConfig();
                
                config.idleMode(IdleMode.kBrake).inverted(INVERTED)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
                
                motor.configure(config,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

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
    public void runPID(double goal) {
        motor.set(pidController.calculate(0,goal));
    }

    @Override
    public void runPIDWithFF(double goal, double velocity) {
        motor.set(pidController.calculate(0,goal) + feedforward.calculate(velocity));
    }
}
