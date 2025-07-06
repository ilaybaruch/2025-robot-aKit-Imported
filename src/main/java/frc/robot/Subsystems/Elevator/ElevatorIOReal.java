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
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class ElevatorIOReal implements ElevatorIO {

    SparkMax motor;
    ProfiledPIDController pidController;
    ElevatorFeedforward feedforward;
    RelativeEncoder encoder;
    POMDigitalInput limitSwitch;

    public ElevatorIOReal() {
        motor = new SparkMax(0, MotorType.kBrushless);
        pidController = new ProfiledPIDController(0, 0, 0, null);
        feedforward = new ElevatorFeedforward(0, 0, 0, 0);
        limitSwitch = new POMDigitalInput(0, false);
        encoder = motor.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();

        // config.idleMode(IdleMode.kCoast).inverted(INVERTED)
        // .smartCurrentLimit(CURRENT_LIMIT)
        // .voltageCompensation(VOLTAGE_COMPENSATION);

        // config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
        // .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
        // motor.configure(config, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);

        // encoder.setPosition(0);

    }
}
