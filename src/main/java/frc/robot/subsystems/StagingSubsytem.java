package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class StagingSubsytem extends SubsystemBase {

    private CANSparkMax bottomIntakeMotor;
    private CANSparkMax topIntakeMotor;

    // .4-3.1 V between 80cm - 10cm
    private final AnalogInput rangeFinder = new AnalogInput(0);

    public StagingSubsytem() {

        this.bottomIntakeMotor = new CANSparkMax(MotorIds.kBottomIntakeMotorCanId, MotorType.kBrushed);
        this.topIntakeMotor = new CANSparkMax(MotorIds.kTopIntakeMotor, MotorType.kBrushed);

        bottomIntakeMotor.restoreFactoryDefaults();
        topIntakeMotor.restoreFactoryDefaults();
    }

    public boolean isNoteInside() {
        return rangeFinder.getValue() > 1000;
    }

    public void stopMotor() {
        bottomIntakeMotor.set(0);
        topIntakeMotor.set(0);
    }

    public void setMotor(double speed) {
        bottomIntakeMotor.set(-1 * speed);
        topIntakeMotor.set(-1 * speed);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("isNoteInside", isNoteInside());
        SmartDashboard.putNumber("Range Finder", rangeFinder.getValue());
    }

}
