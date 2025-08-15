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

        io.setDriveVoltage(state.speedMetersPerSecond / wheelRadiusMeters /* 12 */);

        io.setTurnPosition(state.angle);

    }

    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        io.setDriveVoltage(output);
        io.setTurnPosition(new Rotation2d());
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /** Returns the module velocity in rad/sec. */
    public double getFFCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

}
