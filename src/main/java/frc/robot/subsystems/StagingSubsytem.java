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

public class StagingSubsytem extends SubsystemBase {

    private CANSparkMax bottomIntakeMotor;
    private CANSparkMax topIntakeMotor;
    private CANSparkMax conveyorMotor;

    // .4-3.1 V between 80cm - 10cm
    private final AnalogInput rangeFinder = new AnalogInput(0);

    public StagingSubsytem() {

        this.bottomIntakeMotor = new CANSparkMax(MotorIds.kBottomIntakeMotorCanId, MotorType.kBrushed);
        this.topIntakeMotor = new CANSparkMax(MotorIds.kTopIntakeMotor, MotorType.kBrushed);
        this.conveyorMotor = new CANSparkMax(MotorIds.kConveyorMotor, MotorType.kBrushed);

        bottomIntakeMotor.setInverted(true);
        topIntakeMotor.setInverted(true);
    }

    public boolean isNoteInside() {
        return rangeFinder.getValue() > 1000;
    }

    public void stopMotor() {
        bottomIntakeMotor.set(0);
        topIntakeMotor.set(0);
        conveyorMotor.set(0);
    }

    public void setMotor(double speed) {
        bottomIntakeMotor.set(speed);
        topIntakeMotor.set(speed);
        if (speed < 0) {
            conveyorMotor.set(-1 * SmartDashboard.getNumber("Conveyor Speed", 1.0));
        } else {
            conveyorMotor.set(SmartDashboard.getNumber("Conveyor Speed", 1.0));
        }
    }

    @Override
    public void periodic() {
        super.periodic();
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
}
