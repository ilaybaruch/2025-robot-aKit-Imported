package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

import java.time.Period;

import org.littletonrobotics.junction.Logger;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    private final Timer timer = new Timer();

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("drive/module " + index, inputs);
        // io.setPidValues();
        if (timer.advanceIfElapsed(5)) {
            io.setMotorEncouderToCAN();
        }
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

    public void setAngle(Rotation2d angle) {
        io.setTurnPosition(angle);
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

    public void driveTune() {
        io.setPidValues();
    }

}
