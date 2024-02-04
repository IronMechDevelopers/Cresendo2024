package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class StagingSubsytem extends SubsystemBase {

    private CANSparkMax stagingMotor;
    private final RelativeEncoder m_stagingMotorEncoder;

    // .4-3.1 V between 80cm - 10cm
    private final AnalogInput rangeFinder = new AnalogInput(0);

    public StagingSubsytem() {

        this.stagingMotor = new CANSparkMax(1, MotorType.kBrushless);

        stagingMotor.restoreFactoryDefaults();
        m_stagingMotorEncoder = stagingMotor.getEncoder();
    }

    public boolean isNoteInside() {
        return rangeFinder.getValue() > 1000;
    }

    public void stopMotor() {
        stagingMotor.set(0);
    }

    public void setMotor(double speed) {
        stagingMotor.set(speed);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("isNoteInside", isNoteInside());
        SmartDashboard.putNumber("Range Finder", rangeFinder.getValue());
    }

}
