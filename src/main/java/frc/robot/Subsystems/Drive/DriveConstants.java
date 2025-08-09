package frc.robot.Subsystems.Drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

        public static final double MAX_ACCELERATION_XY = 4;
        public static final double MAX_VELOCETY_XY = 4;
        public static final double TRANSLATION_TOLERANCE = 0.03;

        public static final double MAX_ACCELERATION_OMEGA = 2;
        public static final double MAX_VELOCETY_OMEGA = 1.8;
        public static final double OMEGA_TOLERANCE = 0.05;

        public static final double maxSpeedMetersPerSec = 6;
        public static final double maxAccMetersPerSecSquared = 4;
        public static final double maxSpeedRadiansPerSec = 4;
        public static final double maxAccRadiansPerSecSquared = 4;
        public static final double odometryFrequency = 50.0; // Hz
        public static final double trackWidth = 0.55735;
        public static final double wheelBase = 0.55735;
        public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

        // Zeroed rotation values for each module, see setup instructions
        public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.9215 + Math.PI);
        public static final Rotation2d frontRightZeroRotation = new Rotation2d(
                        0.39 + 3.063 + 2.586 + 2.003);// 0.39
        public static final Rotation2d backLeftZeroRotation = new Rotation2d(1.54 - 0.035 + Math.PI);
        public static final Rotation2d backRightZeroRotation = new Rotation2d(3.05 - 0.05 + 1.015 + Math.PI + 0.218);

        // Device CAN IDs
        public static final int pigeonCanId = 2;

        public static final int swerveBaseID = 3;
        public static final int swerveModuleIDsCount = 3;

        public static final int frontLeftDefaultCanId = 3;
        public static final int backLeftDefaultCanId = 9;
        public static final int frontRightDefaultCanId = 6;
        public static final int backRightDefaultCanId = 13;

        // Drive motor configuration
        public static final int driveMotorCurrentLimit = 50;
        public static final double wheelRadiusMeters = Units.inchesToMeters(2);
        public static final double driveMotorReduction = 5.96; // Change to 5.96 (or 5.46) if using faster ration
        public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

        // Drive encoder configuration
        public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction / (3.19 / 16.523); // Rotor
                                                                                                                     // Rotations
                                                                                                                     // ->
        // Wheel Radians
        public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM
                                                                                                            // ->
                                                                                                            // Wheel
                                                                                                            // Rad/Sec

        public static final double driveSlipCurrent = 70;
        public static final double driveRampRate = 0.22;

        // Turn motor configuration
        public static final boolean turnInverted = true;
        public static final int turnMotorCurrentLimit = 20;
        public static final double turnMotorRampRate = 0.4;
        public static final double turnMotorReduction = 396.0 / 35.0;
        public static final DCMotor turnGearbox = DCMotor.getNEO(1);

        // Turn encoder configuration
        public static final boolean turnEncoderInverted = true;
        public static final double turnEncoderPositionFactor = (2 * Math.PI) / turnMotorReduction; // Rotations ->
                                                                                                   // Radians
        public static final double turnEncoderVelocityFactor = turnEncoderPositionFactor / 60.0; // RPM -> Rad/Sec

        // PathPlanner configuration
        public static final double robotMassKg = 40;
        public static final double robotMOI = 1 / 12.0 * robotMassKg * (2 * trackWidth * trackWidth);
        public static final double wheelCOF = 1.2;
        public static final RobotConfig ppConfig = new RobotConfig(
                        robotMassKg,
                        robotMOI,
                        new ModuleConfig(
                                        wheelRadiusMeters,
                                        maxSpeedMetersPerSec,
                                        wheelCOF,
                                        driveGearbox.withReduction(driveMotorReduction),
                                        driveMotorCurrentLimit,
                                        1),
                        moduleTranslations);

        // turnPID
        public static final double turnKs = 0.22;

}