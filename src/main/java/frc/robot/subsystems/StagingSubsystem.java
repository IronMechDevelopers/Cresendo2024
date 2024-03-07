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
    private final AnalogInput lowerIntakeSensor = new AnalogInput(1);
    private final AnalogInput upperIntakeSensor = new AnalogInput(0);

    public StagingSubsystem() {

        this.bottomIntakeMotor = new CANSparkMax(MotorIds.kBottomIntakeMotorCanId, MotorType.kBrushed);
        this.topIntakeMotor = new CANSparkMax(MotorIds.kTopIntakeMotor, MotorType.kBrushed);
        this.conveyorMotor = new CANSparkMax(MotorIds.kConveyorMotor, MotorType.kBrushed);

        bottomIntakeMotor.setInverted(true);
        topIntakeMotor.setInverted(true);
        currentPercentage = 0;

    }

    public boolean isNoteAtUpperSensor() {
        boolean noteInside = upperIntakeSensor.getValue() > 1000;
        SmartDashboard.putNumber("Upper Sensor", upperIntakeSensor.getValue());
        return noteInside;
    }

    public boolean isNoteAtLowerSensor() {
        boolean noteInside = upperIntakeSensor.getValue() > 1000;
        SmartDashboard.putNumber("Lower Sensor", lowerIntakeSensor.getValue());
        return noteInside;
    }

    public boolean isNoteInside() {
        
        return isNoteAtUpperSensor() || isNoteAtLowerSensor();
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
        SmartDashboard.putBoolean("isNoteInside", isNoteInside());
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
                .until(() -> isNoteAtUpperSensor()), runOuttakeCommand().withTimeout(.25));

    }

}
