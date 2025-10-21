
package frc.robot.Subsystems.Elevator;

import static frc.robot.Subsystems.Elevator.ElevatorConstants.CURRENT_LIMIT;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.INVERTED;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Ka;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Kd;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Kg;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Ki;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Kp;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Ks;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Kv;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.MAX_ACCELERATION;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.MAX_VELOCITY;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.MOTOR_ID;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.TOLERANCE;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.UP_POS;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.VOLTAGE_COMPENSATION;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class ElevatorSparkMax implements ElevatorIO {

    private final ElevatorTuning pidConstants;
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final POMDigitalInput limitSwitch;
    private final ProfiledPIDController pidController;
    private ElevatorFeedforward feedforward;

    public ElevatorSparkMax() {
        limitSwitch = new POMDigitalInput(1); // TODO constant
        pidConstants = new ElevatorTuning();
        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        feedforward = new ElevatorFeedforward(Ks, Kg, Kv, Ka);
        pidController = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

        SparkBaseConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast).inverted(INVERTED)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        config.encoder.positionConversionFactor(1.0)
                .velocityConversionFactor(1.0 / 60.0);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0);

        pidController.setTolerance(TOLERANCE);
        setPidValues();

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.velocity = encoder.getVelocity();
        inputs.voltage = (motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.limitSwitch = limitSwitch.get();
        inputs.Position = encoder.getPosition();
        inputs.atGoal = pidController.atGoal();
        inputs.currentOutput = motor.getAppliedOutput();
    }

    @Override
    public double getFeedForward(double velocity) {
        double voltage = feedforward.calculate(velocity);
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
    public void setFeedForward(double goal, double velocity) {
        motor.setVoltage(getFeedForward(velocity));
    }

    @Override
    public void runPID(double goal) {
        motor.set(pidController.calculate(encoder.getPosition(), goal));
    }

    @Override
    public void runPIDWithFF(double goal, double velocity) {
        motor.setVoltage(pidController.calculate(encoder.getPosition(), goal)
                + getFeedForward(pidController.getSetpoint().velocity));
    }

    @Override
    public boolean isPressed() {
        return limitSwitch.get();
    }

    @Override
    public void resetEncouderIfNeeded() {
        if (isPressed()) {
            encoder.setPosition(0);
        }
    }

    @Override
    public double getPos() {
        return encoder.getPosition();
    }

    @Override
    public boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public void runFF(double velocity) {
        motor.setVoltage(getFeedForward(velocity));
    }

    @Override
    public void resetPID() {
        pidController.reset(encoder.getPosition());
    }

    @Override
    public void setPidValues() {
        pidController.setP(pidConstants.getKp());
        pidController.setI(pidConstants.getKi());
        pidController.setD(pidConstants.getKd());
        pidController.setConstraints(
                new TrapezoidProfile.Constraints(pidConstants.getMaxVelocity(), pidConstants.getMaxAcceleration()));
        feedforward = new ElevatorFeedforward(pidConstants.getKs(), pidConstants.getKg(), pidConstants.getKv(),
                pidConstants.getKa());

        Logger.recordOutput("current kp", pidController.getP());
        Logger.recordOutput("current kg", feedforward.getKg());
        Logger.recordOutput("current ks", feedforward.getKs());

    }
}
