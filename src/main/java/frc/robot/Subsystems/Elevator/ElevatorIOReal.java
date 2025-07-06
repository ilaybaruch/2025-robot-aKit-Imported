package frc.robot.Subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.POM_lib.sensors.POMDigitalInput;

import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {

    SparkMax motor;
    ProfiledPIDController pidController;
    ElevatorFeedforward feedforward;
    RelativeEncoder encoder;
    POMDigitalInput limitSwitch;
    ElevatorTune pidConstants;

    public ElevatorIOReal() {
        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        pidController = new ProfiledPIDController(kP, kI, kD, null);
        feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
        limitSwitch = new POMDigitalInput(LIMIT_SWITCH_ID, IS_SWITCH_OPEN);
        encoder = motor.getEncoder();
        pidConstants = new ElevatorTune();
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast).inverted(INVERTED)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
        motor.configure(config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        encoder.setPosition(0);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.limitSwitch = getLimitSwitch();
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();
        inputs.voltage = (motor.getBusVoltage() * motor.getAppliedOutput());
    }

    @Override
    public void setGoal(double goal) {
        pidController.setGoal(goal);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setSpeed(double precentage) {
        motor.set(precentage);
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    @Override
    public double getPos() {
        return encoder.getPosition();
    }

    @Override
    public void setFF(double velocity, double goal) {
        setGoal(goal);
        motor.setVoltage(feedforward.calculate(velocity));
    }

    @Override
    public void setPIDWithFF(double goal, double velocity) {
        setGoal(goal);
        motor.setVoltage(feedforward.calculate(velocity) + pidController.calculate(encoder.getPosition(), goal));
    }

    @Override
    public void setPIDvalues() {
        pidController.setP(pidConstants.getKp());
        pidController.setI(pidConstants.getKi());
        pidController.setD(pidConstants.getKd());
        pidController.setConstraints(
                new TrapezoidProfile.Constraints(pidConstants.getMaxVelocity(), pidConstants.getMaxAcceleration()));
        feedforward = new ElevatorFeedforward(pidConstants.getKs(), pidConstants.getKg(), pidConstants.getKv(),
                pidConstants.getKa());

    }
}
