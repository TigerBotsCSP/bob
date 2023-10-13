package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;

public class Arm {
    private CANSparkMax m_leftIntaker;
    private CANSparkMax m_rightIntaker;

    // Public for testing encoders
    public CANSparkMax m_rotation;
    public CANSparkMax m_rightRotation;

    public boolean m_in = false;

    Arm() {
        m_leftIntaker = new CANSparkMax(11, MotorType.kBrushless);
        m_rightIntaker = new CANSparkMax(10, MotorType.kBrushless);
        //Right spins inverse of left
        m_rightIntaker.follow(m_leftIntaker, true);

        m_rotation = new CANSparkMax(13, MotorType.kBrushless);
    }

    // Toggle the intaker
    // If it's going in, make it go out, and vise-versa
    public void toggleIntaker() {
        if (m_in) {
            m_leftIntaker.set(ArmConstants.kIntakerSpeed);
            m_in = false;
        } else {
            m_leftIntaker.set(-ArmConstants.kIntakerSpeed);
            m_in = true;
        }
    }

    // Set the rotation of the arm using the joystick value
    public void setRotation(double speed) {
        m_rotation.set(-Math.max(Math.min(speed, ArmConstants.kRotationSpeed), -ArmConstants.kRotationSpeed));
    }
}