package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {

    private static final int kCanID = 13;
    private static final MotorType kMotorType = MotorType.kBrushless;
    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int kCPR = 8192;
  
  
    private CANSparkMax m_motor;
    private SparkMaxPIDController m_pidController;
  
    /**
     * An alternate encoder object is constructed using the GetAlternateEncoder() 
     * method on an existing CANSparkMax object. If using a REV Through Bore 
     * Encoder, the type should be set to quadrature and the counts per 
     * revolution set to 8192
     */
    private RelativeEncoder m_alternateEncoder;

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
    //public CANSparkMax m_rotation;

    public IntakerMode m_currentMode = IntakerMode.IDLE;

    public ArmSubsystem() {

    // initialize SPARK MAX with CAN ID
    m_motor = new CANSparkMax(kCanID, kMotorType);
    m_motor.restoreFactoryDefaults();
    //set controller to use through-bore (absolute) encoder 
    m_alternateEncoder = m_motor.getAlternateEncoder(kAltEncType, kCPR);
    m_pidController = m_motor.getPIDController();
    m_pidController.setFeedbackDevice(m_alternateEncoder);

    // set PID coefficients
    m_pidController.setP(.1);
    m_pidController.setI(1e-4);
    m_pidController.setD(1);
    m_pidController.setIZone(0);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(-.2, .2);


        m_intaker = new CANSparkMax(11, MotorType.kBrushless);
        m_followIntaker = new CANSparkMax(10, MotorType.kBrushless);
        //Right automatically spins with left
        m_followIntaker.follow(m_intaker, true);

        // m_rotation = new CANSparkMax(13, MotorType.kBrushless);
        // //Stop arm from falling
        // m_rotation.setIdleMode(IdleMode.kBrake);
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

    // // Set the rotation of the arm using the joystick value
    // public void setRotationSpeed(double speed) {
    //     m_rotation.set(-Math.max(Math.min(speed, ArmConstants.kRotationSpeed), -ArmConstants.kRotationSpeed));
    // }

    // Toggle the intaker
    // If it's going in, make it go out, and vise-versa
    public void ArmShootPosition() {
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

    public Command setArmPosition(double rotations){
        return this.runOnce(() -> m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition));
    }

}