package frc.robot.Subsystems.Drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        double driveVelocity;
        double turnVelocity;
        double turnPosition;
        double driveVoltage;
        double turnVoltage;
        double driveOutput;
        double turnOutput;
    }

    public default void setVelocity(ChassisSpeeds speeds) {
    }

    public default void setDriveVelocity(double speed) {
    }

    public default void setDriveVoltage(double voltage) {
    }

    public default void setTurnSpeed(double speed) {
    }

    public default void setTurnVoltage(double voltage) {
    }

    public default void stopMotors() {
    }

    public default void setTurnPosition(Rotation2d setpoint){

    }

    public default double getAbsolutePosition(){
        return 0;
    }

    public default Rotation2d getAbsolutePositionRotation2d(){
        return new Rotation2d();
    }



    public default Rotation2d getEncouderAngle() {
        return new Rotation2d();
    }
}
