package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

public class DrivePIDTuning {
    LoggedNetworkNumber turnKpTune = new LoggedNetworkNumber("KP", turnKp);
    LoggedNetworkNumber turnKdTune = new LoggedNetworkNumber("KD", turnKd);
    LoggedNetworkNumber turnKsTune = new LoggedNetworkNumber("KS", turnKs);
    LoggedNetworkNumber turnKffTune = new LoggedNetworkNumber("KS", turnKff);

    public double getKp() {
        return turnKpTune.get();
    }

    public double getKd() {
        return turnKdTune.get();
    }

    public double getKs() {
        return turnKsTune.get();
    }

    public double getKff() {
        return turnKffTune.get();
    }

}
