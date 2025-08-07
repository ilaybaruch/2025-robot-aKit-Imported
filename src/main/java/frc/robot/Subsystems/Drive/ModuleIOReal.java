package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.POM_lib.Motors.POMTalonSRX;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

import java.util.function.DoubleSupplier;

public class ModuleIOReal implements ModuleIO {
    private final int module;
    private final TalonFX driveMotor;
    private final SparkMax turnMotor;
    private final CANcoder turnEncoder;
    private final SparkClosedLoopController turnController;

    private final VelocityVoltage velocityVoltageRequest;
    // private final Translation2d frontLeftLocation;
    // private final Translation2d frontRightLocation;
    // private final Translation2d backLeftLocation;
    // private final Translation2d backRightLocation;
    // private final SwerveDriveKinematics kinematics;

    public ModuleIOReal(int module) {
        this.module = module;

        Rotation2d zeroRotation = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 -> frontRightZeroRotation;
            case 2 -> backLeftZeroRotation;
            case 3 -> backRightZeroRotation;
            default -> new Rotation2d();
        };

        // frontLeftLocation = new Translation2d(0.65, 0.65);
        // frontRightLocation = new Translation2d(0.65, -0.65);
        // backLeftLocation = new Translation2d(-0.65, 0.65);
        // backRightLocation = new Translation2d(-0.65, -0.65);

        driveMotor = new TalonFX(swerveBaseID + swerveModuleIDsCount * module);
        turnMotor = new SparkMax(swerveBaseID + 1 + swerveModuleIDsCount * module, MotorType.kBrushless);

        turnEncoder = new CANcoder(swerveBaseID + 2 + swerveModuleIDsCount * module);
        turnController = turnMotor.getClosedLoopController();

        velocityVoltageRequest = new VelocityVoltage(0.0);

        // encouder config
        var encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = module == 1 ? SensorDirectionValue.CounterClockwise_Positive
                : SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = zeroRotation.getRotations();

        // drive motor config
        var driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveConfig.Feedback.SensorToMechanismRatio = driveEncoderPositionFactor;
        driveConfig.Feedback.RotorToSensorRatio = 1.0;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveSlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveSlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = driveSlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = driveRampRate;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = driveRampRate;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = driveRampRate;

        // turn motor config
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(turnInverted)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12.0)
                .closedLoopRampRate(turnMotorRampRate)
                .openLoopRampRate(turnMotorRampRate);
        turnConfig.encoder
                .positionConversionFactor(turnEncoderPositionFactor)
                .velocityConversionFactor(turnEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .outputRange(-0.25, 0.25);
        turnConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

    }

    @Override
    public void stopMotors() {
        turnMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    public Rotation2d getEncouderAngle() {
        return new Rotation2d(turnEncoder.getPosition().getValueAsDouble());
    }

    @Override
    public void setDriveVelocity(double velocity) {
        double velocityRotPerSec = Units.radiansToRotations(velocity);
        driveMotor.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
    }

    @Override
    public double getAbsolutePosition() {
        return turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }

    @Override
    public Rotation2d getAbsolutePositionRotation2d() {
        return Rotation2d.fromRadians(getAbsolutePosition());
    }

    @Override
    public void setTurnPosition(Rotation2d setpoint) {
        double error = setpoint.minus(Rotation2d.fromRadians(getAbsolutePosition())).getRadians();
    if (Math.abs(error) >= Math.PI) {
      error -= Math.copySign(Math.PI, error);
      error *= -1;
    }
    var ks = Math.copySign(turnKs, error);
    if (Math.abs(error) > 0.03) {
      turnController.setReference(setpoint.getRadians(), ControlType.kPosition,
          ClosedLoopSlot.kSlot0, ks, ArbFFUnits.kVoltage);
    } else {
      setTurnVoltage(0);
    }
    }
}