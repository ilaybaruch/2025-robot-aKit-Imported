package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog

    public static class ArmIOInputs {
        double voltage;
        double velocity;
        boolean downLimitSwitch;
        boolean upperLimitSwitch;
        boolean atGoal;
        double currentOutput;

    }

    public default void updateInputs(ArmIOInputs inputs) {
    }

    public default void setSpeed(double precentage) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void stopArm() {
    }

    public default void runPID(double goal) {
    }

    public default void runPIDWithFF(double goal, double velocity) {
    }

    public default void runFF(double velocity) {
    }

    public default void setPidValues() {
    }

    public default boolean downIsPressed() {
        return false;
    }

    public default boolean upperIsPressed() {
        return false;
    }

    public default void resetUpperEncouderIfNeeded() {
    }

    public default void resetDownEncouderIfNeeded() {
    }

    public default void resetPID() {
    }

    public default double getPos() {
        return 0;
    }

    public default boolean atGoal() {
        return false;
    }

}
