package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class StagingSubsystem extends SubsystemBase {

    private CANSparkMax bottomIntakeMotor;
    private CANSparkMax topIntakeMotor;
    private CANSparkMax conveyorMotor;
    private double currentPercentage;

    // .4-3.1 V between 80cm - 10cm
    private final AnalogInput rangeFinder = new AnalogInput(0);

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    public StagingSubsystem() {

        this.bottomIntakeMotor = new CANSparkMax(MotorIds.kBottomIntakeMotorCanId, MotorType.kBrushed);
        this.topIntakeMotor = new CANSparkMax(MotorIds.kTopIntakeMotor, MotorType.kBrushed);
        this.conveyorMotor = new CANSparkMax(MotorIds.kConveyorMotor, MotorType.kBrushed);

        bottomIntakeMotor.setInverted(true);
        topIntakeMotor.setInverted(true);
        currentPercentage = 0;

        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(9);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(36);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 0, 255, 0);
        }

        m_led.setData(m_ledBuffer);

    }

    private void setColor(int red, int blue, int green) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, red, blue, green);
        }

        m_led.setData(m_ledBuffer);
    }

    public boolean isNoteInside() {
        boolean noteInside = rangeFinder.getValue() > 1000;
        SmartDashboard.putBoolean("Note Inside", noteInside);
        SmartDashboard.putNumber("Range Finder", rangeFinder.getValue());
        if (noteInside) {
            setColor(0, 0, 255);
        }
        return noteInside;
    }

    public void stopMotor() {
        bottomIntakeMotor.set(0);
        topIntakeMotor.set(0);
        conveyorMotor.set(0);
        currentPercentage = 0;
    }

    public void setMotor(double speed) {
        bottomIntakeMotor.set(speed);
        topIntakeMotor.set(speed);
        currentPercentage = speed;
        if (speed < 0) {
            conveyorMotor.set(-1 * SmartDashboard.getNumber("Conveyor Speed", 1.0));
        } else {
            conveyorMotor.set(SmartDashboard.getNumber("Conveyor Speed", 1.0));
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("StagingSubsystem Percentage", currentPercentage);

    }

    public Command runIntakeCommand() {
        return Commands.startEnd(() -> setMotor(Constants.SpeedConstants.IntakeSpeed), () -> stopMotor(), this);
    }

    public Command runOuttakeCommand() {
        return Commands.startEnd(() -> setMotor(Constants.SpeedConstants.OuttakeSpeed), () -> stopMotor(), this);
    }

    public Command drivingIntakeCommand() {
        return Commands.sequence(runIntakeCommand()
                .until(() -> isNoteInside()), runOuttakeCommand().withTimeout(.25));

    }

    public Command changeColorCommmand(int red, int blue, int green) {
        return Commands.runOnce(() -> setColor(red, blue, green));
    }
}
