package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import frc.robot.Subsystems.Drive.Module;

public class Drive extends SubsystemBase {
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final GyroIO gyroIO;
    private final ModuleIOInputsAutoLogged moduleInputs = new ModuleIOInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private SwerveDriveKinematics kinematics;

    public Drive(GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);
        kinematics = new SwerveDriveKinematics(moduleTranslations);
    }


    public void setVelocity(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyroIO.getGyroRotation());
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < 4; i++) {
            modules[i].runSetPoint(setpointStates[i]);
        }

    }

    public void stopDrive(){
        for (int i = 0; i < 4; i++) {
            modules[i].stopModules();;
        }
    }



}
