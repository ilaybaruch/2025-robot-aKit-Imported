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

    public default void updateInputs() {
    }

    public default void setSpeed() {
    }

    public default void setVoltage() {
    }

    public default void setPIDWithFF() {
    }

    public default void setFF() {
    }

    public default void setGoal() {
    }

    public default boolean atGoal() {
        return false;
    }

}