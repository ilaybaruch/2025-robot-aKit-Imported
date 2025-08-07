package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged moduleInputs = new ModuleIOInputsAutoLogged();
    private final int index;

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void runSetPoint(SwerveModuleState state) {
        state.optimize(io.getEncouderAngle());
        state.cosineScale(io.getEncouderAngle());
        io.setDriveVelocity(state.speedMetersPerSecond);
        io.setTurnPosition(state.angle);
    }

    public void stopModules(){
        io.stopMotors();
    }

}
