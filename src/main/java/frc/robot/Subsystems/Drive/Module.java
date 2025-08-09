package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

import java.time.Period;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public void stop() {
        io.stop();
    }

    public void runVelocity(SwerveModuleState state) {
        state.optimize(getAngle());
        state.cosineScale(getAngle());

        io.setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters);

        io.setTurnPosition(state.angle);

    }

    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

}
