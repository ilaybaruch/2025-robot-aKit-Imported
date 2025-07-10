package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface ElevatorIO {

    @AutoLog

    public static class ElevatorIOInputs {
        double voltage;
        double velocity;
        boolean limitSwitch;
        double Position;
        double output;
        Pose3d pose;
        double pidCalc;
        boolean atGoal;

    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setSpeed(double precentage) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void stopElevator() {
    }

    public default void runPID(double goal) {
    }

    public default void runPIDWithFF(double goal, double velocity) {
    }

    public default double getFeedForward(double velocity) {
        return 0;
    }

    public default void setPidValues() {
    }

    public default void setFeedForward(double goal, double velocity) {
    }

    public default boolean ifPressed() {
        return false;
    }

    public default void resetEncouder() {
    }

    public default double getPos() {
        return 0;
    }

    public default boolean atGoal() {
        return false;
    }

    public default void runFF(double velocity) {
    }

}