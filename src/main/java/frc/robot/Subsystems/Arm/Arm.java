package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO armIO) {
        this.armIO = armIO;
    }

    public ArmIO getIO() {
        return armIO;
    }

    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.processInputs("arm", armInputs);
        armIO.setPidValues();
        getIO().resetUpperEncouderIfNeeded();// feels dumb with the getIO and armIO
        getIO().resetDownEncouderIfNeeded();// yep it feels dumb
        SmartDashboard.putString("CurremtCommand",
                getCurrentCommand() == null ? "None" : getCurrentCommand().getName());// cool af

    }
}
