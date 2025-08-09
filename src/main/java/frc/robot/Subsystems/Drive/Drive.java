package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

public class Drive extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL FR BL BR

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    private Rotation2d rawGyroRotation = new Rotation2d();

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);
    }

    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

        if (Math.abs(speeds.omegaRadiansPerSecond + speeds.vxMetersPerSecond + speeds.vyMetersPerSecond) < 0.01) {
            for (int i = 0; i < 4; i++) {
                modules[i].stop();
            }
        } else {
            // Send setpoints to modules
            for (int i = 0; i < 4; i++) {
                modules[i].runVelocity(setpointStates[i]);
            }

        }

    }

    // /** Returns the maximum linear speed in meters per sec. */
    // public double getMaxLinearSpeedMetersPerSec() {
    // return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // }

    // /** Returns the maximum angular speed in radians per sec. */
    // public double getMaxAngularSpeedRadPerSec() {
    // return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    // }
}
