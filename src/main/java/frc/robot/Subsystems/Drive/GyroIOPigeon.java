package frc.robot.Subsystems.Drive;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Subsystems.Drive.DriveConstants.*;

public class GyroIOPigeon implements GyroIO {

    private final WPI_PigeonIMU pigeon;
    private Rotation2d offset;

    public GyroIOPigeon() {
        pigeon = new WPI_PigeonIMU(pigeonCanId);
        offset = new Rotation2d();
        pigeon.reset();
    }

    @Override
    public Rotation2d getGyroRotation() {
        return pigeon.getRotation2d().minus(offset);
    }

    @Override
    public AngularVelocity getGyroAngularVelocity() {
        return AngularVelocity.ofBaseUnits(Units.degreesToRadians(pigeon.getRate()), RadiansPerSecond);
    }

    @Override
    public void reset() {
        reset(new Rotation2d());
    }

    @Override
    public void reset(Rotation2d to) {
        offset = pigeon.getRotation2d().minus(to);
    }
}
