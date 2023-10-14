package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private enum IntakerMode {
        INTAKE(ArmConstants.kIntakerSpeed),
        SHOOT(ArmConstants.kShootSpeed),
        IDLE(0);

        public double m_motorSpeed;

        IntakerMode(double motorSpeed) {
            m_motorSpeed = motorSpeed;
        }
    }

    private CANSparkMax m_intaker;
    private CANSparkMax m_followIntaker;

    // Public for testing encoders
    public CANSparkMax m_rotation;

    public IntakerMode m_currentMode = IntakerMode.IDLE;

    public ArmSubsystem() {
        m_intaker = new CANSparkMax(11, MotorType.kBrushless);
        m_followIntaker = new CANSparkMax(10, MotorType.kBrushless);
        //Right automatically spins with left
        m_followIntaker.follow(m_intaker, true);

        m_rotation = new CANSparkMax(13, MotorType.kBrushless);
        //Stop arm from falling
        m_rotation.setIdleMode(IdleMode.kBrake);
    }

    // Toggle the intaker
    // If it's going in, make it go out, and vise-versa
    public void toggleIntaker() {
        switch (m_currentMode) {
            case IDLE:
            setIntakerMode(IntakerMode.INTAKE);
                break;
            case INTAKE:
            setIntakerMode(IntakerMode.IDLE);
                break;
            default:
                break;
        }
    }

    public Command getToggleIntakerCommand() {
        return this.runOnce(() -> this.toggleIntaker());
    }

    public Command getShootCommand() {
        return this.runOnce(() -> this.setIntakerMode(IntakerMode.SHOOT))
        .andThen(
            new WaitCommand(0.25),
            this.runOnce(() -> this.setIntakerMode(IntakerMode.IDLE)))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    private void setIntakerMode(IntakerMode mode) {
        this.m_currentMode = mode;
        m_intaker.set(m_currentMode.m_motorSpeed);
    }

    // Set the rotation of the arm using the joystick value
    public void setRotationSpeed(double speed) {
        m_rotation.set(-Math.max(Math.min(speed, ArmConstants.kRotationSpeed), -ArmConstants.kRotationSpeed));
    }
}