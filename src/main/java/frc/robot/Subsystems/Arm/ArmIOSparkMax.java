package frc.robot.Subsystems.Arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.POM_lib.sensors.POMDigitalInput;

import static frc.robot.Subsystems.Arm.ArmConstants.*;

public class ArmIOSparkMax implements ArmIO {

    private final SparkMax motor;
    private final POMDigitalInput upperSwitch;
    private final POMDigitalInput downSwitch;
    private final RelativeEncoder encoder;
    private final ProfiledPIDController pidController;
    private ArmFeedforward feedforward;
    private final ArmTuning tuning;

    public ArmIOSparkMax() {
        motor = new SparkMax(0, MotorType.kBrushless);
        encoder = motor.getEncoder();
        upperSwitch = new POMDigitalInput(0);
        downSwitch = new POMDigitalInput(0);
        pidController = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        feedforward = new ArmFeedforward(Ks, Kg, Kv);
        tuning = new ArmTuning();

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
    public void updateInputs(ArmIOInputs inputs) {
        inputs.atGoal = atGoal();
        inputs.velocity = motor.get();
        inputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();// hatol
        inputs.currentOutput = motor.getAppliedOutput();
        inputs.upperLimitSwitch = upperIsPressed();
        inputs.downLimitSwitch = downIsPressed();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setSpeed(double precentage) {
        motor.set(precentage / 100); // dont know if i want to keep dis shit
    }

    @Override
    public void stopArm() {
        motor.stopMotor();
    }

    @Override
    public boolean upperIsPressed() {
        return upperSwitch.get();
    }

    @Override
    public boolean downIsPressed() {
        return downSwitch.get();
    }

    @Override
    public void runPID(double goal) {
        setVoltage(pidController.calculate(0, goal)); // TODO check positions and yeh
    }

    @Override
    public void runFF(double velocity) {
        setVoltage(feedforward.calculate(0, 0)); // TODO check positions and yeh (agav hatol)
    }

    @Override
    public void runPIDWithFF(double goal, double velocity) {
        setVoltage(pidController.calculate(0, goal) + feedforward.calculate(0, pidController.getSetpoint().velocity)); // TODO
                                                                                                                       // check
                                                                                                                       // positions
                                                                                                                       // and
                                                                                                                       // yeh
                                                                                                                       // (agav
                                                                                                                       // agav
                                                                                                                       // shney
                                                                                                                       // hatolim)
    }

    @Override
    public void resetUpperEncouderIfNeeded() {
        if (upperIsPressed()) {
            encoder.setPosition(UP_POS); // check positon hell nahhhhhhhhhhhhhhhhh
        }
    }

    @Override
    public void resetDownEncouderIfNeeded() {
        if (downIsPressed()) {
            encoder.setPosition(0); // not need to check position hell yeahhhhhhhhhhhhhhhhhhhhhhhhhh
        }
    }

    @Override
    public void resetPID() {
        pidController.reset(encoder.getPosition()); // TODO check positions and yeh (agav hatol) [lo haya li koah
                                                    // lahshov al masheho hadash]
    }

    @Override
    public double getPos() {
        return encoder.getPosition(); // TODO pos
    }

    @Override
    public boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public void setPidValues() {
        pidController.setP(tuning.getKp());
        pidController.setI(tuning.getKi());
        pidController.setD(tuning.getKd());
        pidController.setConstraints(
                new TrapezoidProfile.Constraints(tuning.getMaxVelocity(), tuning.getMaxAcceleration()));
        feedforward = new ArmFeedforward(tuning.getKs(), tuning.getKg(), tuning.getKv(),
                tuning.getKa());
    }
}
