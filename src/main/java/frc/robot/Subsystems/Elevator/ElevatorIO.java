package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog

    public static class ElevatorIOInputs {
        public double voltage;
        public double velocity;
        public double position;
        public boolean limitSwitch;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setSpeed(double precentage) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void stopMotor() {
    }

    public default void setPIDWithFF(double goal, double velocity) {
    }

    public default void setFF(double velocity, double goal) {
    }

    public default void setGoal(double goal) {
    }

    public default boolean atGoal() {
        return false;
    }

    public default boolean getLimitSwitch() {
        return false;
    }

    public default double getPos() {
        return 0;
    }

    public default void setPIDvalues() {
    }

}