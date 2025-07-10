package frc.robot.Subsystems.Elevator;

import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorTune {

    LoggedNetworkNumber kpTune = new LoggedNetworkNumber("kp", kP);
    LoggedNetworkNumber kiTune = new LoggedNetworkNumber("ki", kI);
    LoggedNetworkNumber kdTune = new LoggedNetworkNumber("kd", kD);
    LoggedNetworkNumber kgTune = new LoggedNetworkNumber("kg", kG);
    LoggedNetworkNumber kaTune = new LoggedNetworkNumber("ka", kA);
    LoggedNetworkNumber ksTune = new LoggedNetworkNumber("ks", kS);
    LoggedNetworkNumber kvTune = new LoggedNetworkNumber("kv", kV);
    LoggedNetworkNumber maxAcceleration = new LoggedNetworkNumber("max acceleration", MAX_ACCELERATION);
    LoggedNetworkNumber maxVelocity = new LoggedNetworkNumber("max velocity", MAX_VELOCITY);
    LoggedNetworkNumber upperKg = new LoggedNetworkNumber("upper gk", UPPER_KG);

    public double getKp() {
        return kpTune.get();
    }

    public double getKi() {
        return kiTune.get();
    }

    public double getKd() {
        return kdTune.get();
    }

    public double getKv() {
        return kvTune.get();
    }

    public double getKa() {
        return kaTune.get();
    }

    public double getKg() {
        return kgTune.get();
    }

    public double getKs() {
        return ksTune.get();
    }

    public double getMaxAcceleration() {
        return maxAcceleration.get();
    }

    public double getMaxVelocity() {
        return maxVelocity.get();
    }

    public double getUpperKg() {
        return upperKg.get();
    }
}
