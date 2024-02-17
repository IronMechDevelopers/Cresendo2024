package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class StagingSubsytem extends SubsystemBase {

    private CANSparkMax bottomIntakeMotor;
    private CANSparkMax topIntakeMotor;
    private CANSparkMax conveyorMotor;

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    // .4-3.1 V between 80cm - 10cm
    private final AnalogInput rangeFinder = new AnalogInput(0);

    // line break sensor code
    //private final DigitalInput laser = new DigitalInput(1);

    public StagingSubsytem() {

        this.bottomIntakeMotor = new CANSparkMax(MotorIds.kBottomIntakeMotorCanId, MotorType.kBrushed);
        this.topIntakeMotor = new CANSparkMax(MotorIds.kTopIntakeMotor, MotorType.kBrushed);
        this.conveyorMotor = new CANSparkMax(MotorIds.kConveyorMotor, MotorType.kBrushed);

        // bottomIntakeMotor.restoreFactoryDefaults();
        // topIntakeMotor.restoreFactoryDefaults();
        // conveyorMotor.restoreFactoryDefaults();

        bottomIntakeMotor.setInverted(true);
        topIntakeMotor.setInverted(true);

        m_led = new AddressableLED(9);

        m_ledBuffer = new AddressableLEDBuffer(36);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setColor(int red, int blue, int green) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, red, green, blue);
        }

        m_led.setData(m_ledBuffer);
    }

    public boolean isNoteInside() {

        if (rangeFinder.getValue() > 1000) {
             System.out.println("NoteInside");
            return (true);
        } else {
            System.out.println("NoNoteInside");
            System.out.println(rangeFinder.getValue());
            return (false);
            
        }

        // line break sensor code
        // return laser.getValue () > 0;

    }

    public void stopMotor() {
        bottomIntakeMotor.set(0);
        topIntakeMotor.set(0);
        conveyorMotor.set(0);
    }

    public void setMotor(double speed) {
        bottomIntakeMotor.set(speed);
        topIntakeMotor.set(speed);
        conveyorMotor.set(SmartDashboard.getNumber("Conveyor Speed", 1.0));
        

    }

    

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("isNoteInside", isNoteInside());
        SmartDashboard.putNumber("Range Finder", rangeFinder.getValue());
        //SmartDashboard.putBoolean("BREAK SENSOR", laser.get());
        SmartDashboard.putNumber("Conveyor Speed", 1.0);
        // System. out. println("periodic trace");

    }

}
