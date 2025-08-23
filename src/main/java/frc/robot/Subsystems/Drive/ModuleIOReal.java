package frc.robot.Subsystems.Drive;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.reduxrobotics.sensors.canandcolor.CanandcolorDetails.Enums.SlotOpcode;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

import org.littletonrobotics.junction.Logger;

public class ModuleIOReal implements ModuleIO {
        private final int module;

        private final TalonFX driveMotor;
        private final SparkMax turnMotor;

        private final CANcoder turnEncouder;
        private final SparkClosedLoopController turnController;

        private final VelocityVoltage velocityVoltage;
        private final SparkMaxConfig turnConfig;

        private final DrivePIDTuning drivePIDTuning;

        public ModuleIOReal(int module) {
                this.module = module;

                Rotation2d zeroRotation = switch (module) {
                        case 0 -> frontLeftZeroRotation;
                        case 1 -> frontRightZeroRotation;
                        case 2 -> backLeftZeroRotation;
                        case 3 -> backRightZeroRotation;
                        default -> new Rotation2d();
                };

                driveMotor = new TalonFX(swerveBaseID + swerveModuleIDsCount * module);
                turnMotor = new SparkMax(swerveBaseID + 1 + swerveModuleIDsCount * module, MotorType.kBrushless);
                turnConfig = new SparkMaxConfig();

                turnEncouder = new CANcoder(swerveBaseID + 2 + swerveModuleIDsCount * module);
                turnController = turnMotor.getClosedLoopController();

                velocityVoltage = new VelocityVoltage(0);

                drivePIDTuning = new DrivePIDTuning();

                // encouder config
                var encoderConfig = new CANcoderConfiguration();
                encoderConfig.MagnetSensor.SensorDirection = module == 1
                                ? SensorDirectionValue.CounterClockwise_Positive
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
                turnConfig.closedLoopRampRate(0.5)
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
                                .pidf(turnKp, 0, turnKd, turnKff, ClosedLoopSlot.kSlot1)
                                .positionWrappingEnabled(true)
                                .outputRange(-0.30, 0.30);
                turnConfig.signals
                                .primaryEncoderPositionAlwaysOn(true)
                                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                                .primaryEncoderVelocityAlwaysOn(true)
                                .primaryEncoderVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);

                turnMotor.configure(turnConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
        }

        @Override
        public void updateInputs(ModuleIOInputs inputs) {
                var driveStatus = BaseStatusSignal.refreshAll(
                                driveMotor.getPosition(),
                                driveMotor.getVelocity(),
                                driveMotor.getMotorVoltage(),
                                driveMotor.getStatorCurrent());
                inputs.drivePositionRad = Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble());
                inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble());
                inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();
                inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValueAsDouble();

                inputs.turnPosition = new Rotation2d(getAbsolutePosition());
                inputs.turnCurrentAmps = getAbsolutePosition();

        }

        @Override
        public void setDriveVelocity(double velocity) {
                driveMotor.set(velocity);
        }

        @Override
        public void setDriveVoltage(double voltage) {
                driveMotor.setVoltage(voltage);
        }

        @Override
        public void setTurnVoltage(double voltage) {
                turnMotor.setVoltage(voltage);
        }

        @Override
        public void setTurnPosition(Rotation2d setpoint) {
                Logger.recordOutput("set point" + module, setpoint.getDegrees());
                double error = setpoint.minus(Rotation2d.fromRadians(getAbsolutePosition())).getRadians();
                if (Math.abs(error) >= Math.PI) {
                        error -= Math.copySign(Math.PI, error);
                        error *= -1;
                }
                var ks = Math.copySign(turnKs, error);
                if (Math.abs(error) > 0.03) {
                        turnController.setReference(setpoint.getRadians(), ControlType.kPosition,
                                        ClosedLoopSlot.kSlot1, ks, ArbFFUnits.kVoltage);
                } else {
                        setTurnVoltage(0);
                }

        }

        @Override
        public double getAbsolutePosition() {
                return turnEncouder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        }

        @Override
        public void stop() {
                driveMotor.stopMotor();
                turnMotor.stopMotor();
        }

        // @Override
        // public void setPID() {
        // turnController.setReference(null, null);
        // }

        @Override
        public void setPidValues() {
                turnConfig.closedLoop.pidf(drivePIDTuning.getKp(), 0, drivePIDTuning.getKd(), drivePIDTuning.getKff(),
                                ClosedLoopSlot.kSlot1);
                turnMotor.configure(turnConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
                                PersistMode.kNoPersistParameters);
        }

        @Override
        public void setMotorEncouderToCAN() {
                turnMotor.getEncoder().setPosition(getAbsolutePosition());
        }

}
