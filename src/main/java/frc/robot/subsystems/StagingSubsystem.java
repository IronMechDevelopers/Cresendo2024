package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIds;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class StagingSubsystem extends SubsystemBase {

    private CANSparkMax bottomIntakeMotor;
    private CANSparkMax topIntakeMotor;
    private CANSparkMax conveyorMotor;
    private double currentPercentage;
    private StagingState stagingState;

    public enum StagingState {
        EMPTY, NOTE_INSIDE, DRIVING_INTAKE, ASK_FOR_NOTE
    };

    // .4-3.1 V between 80cm - 10cm
    private final AnalogInput lowerIntakeSensor = new AnalogInput(2);
    private final AnalogInput upperIntakeSensor = new AnalogInput(0);

    public StagingSubsystem() {

        this.bottomIntakeMotor = new CANSparkMax(MotorIds.kBottomIntakeMotorCanId, MotorType.kBrushed);
        this.topIntakeMotor = new CANSparkMax(MotorIds.kTopIntakeMotor, MotorType.kBrushed);
        this.conveyorMotor = new CANSparkMax(MotorIds.kConveyorMotor, MotorType.kBrushed);

        bottomIntakeMotor.setInverted(true);
        topIntakeMotor.setInverted(true);
        currentPercentage = 0;
        stagingState = StagingState.EMPTY;
    }

    public boolean isNoteAtUpperSensor() {
        boolean noteInside = upperIntakeSensor.getValue() > 1000;

        return noteInside;
    }

    public boolean isNoteAtLowerSensor() {
        boolean noteInside = lowerIntakeSensor.getValue() > 1300;

        return noteInside;
    }

    public boolean isNoteInside() {
        // boolean ans = isNoteAtUpperSensor() || isNoteAtLowerSensor();
        boolean ans = isNoteAtUpperSensor();
        if (ans) {
            stagingState = StagingState.NOTE_INSIDE;
        } else if (bottomIntakeMotor.get() == 0) {
            stagingState = StagingState.EMPTY;
        } else {
            stagingState = StagingState.DRIVING_INTAKE;
        }
        return ans;
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

    public StagingState getState() {
        return stagingState;
    }

    public void setState(StagingState stagingState) {
        this.stagingState = stagingState;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Upper Sensor", upperIntakeSensor.getValue());
        SmartDashboard.putNumber("Lower Sensor", lowerIntakeSensor.getValue());
        SmartDashboard.putBoolean("isNoteInside", isNoteInside());
        SmartDashboard.putNumber("StagingSubsystem Percentage", currentPercentage);
        if (this.stagingState == StagingState.ASK_FOR_NOTE) {
            SmartDashboard.putString("stagingState", "ASK_FOR_NOTE");
        } else if (this.stagingState == StagingState.NOTE_INSIDE) {
            SmartDashboard.putString("stagingState", "NOTE_INSIDE");
        } else if (this.stagingState == StagingState.EMPTY) {
            SmartDashboard.putString("stagingState", "EMPTY");
        } else if (this.stagingState == StagingState.DRIVING_INTAKE) {
            SmartDashboard.putString("stagingState", "DRIVING_INTAKE");
        }

    }

    public Command runIntakeCommand() {
        return Commands.startEnd(() -> setMotor(Constants.SpeedConstants.IntakeSpeed), () -> stopMotor(), this);
    }

    public Command runOuttakeCommand() {
        return Commands.startEnd(() -> setMotor(Constants.SpeedConstants.OuttakeSpeed), () -> stopMotor(), this);
    }

    public Command setStagingStateToDrivingIntake() {
        return Commands.runOnce(() -> setState(StagingState.DRIVING_INTAKE));

    }

    public Command drivingIntakeCommand() {
        return Commands.sequence(setStagingStateToDrivingIntake(), runIntakeCommand()
                .until(() -> isNoteAtUpperSensor()), runOuttakeCommand().withTimeout(.25));

    }

    public Command askForNote() {
        return Commands.startEnd(() -> setState(StagingState.ASK_FOR_NOTE), () -> setState(StagingState.EMPTY), this);
    }

}
