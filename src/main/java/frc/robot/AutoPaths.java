package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;

public final class AutoPaths {

    private RobotContainer m_robotContainer;
    private ArmSubsystem m_arm;

    public AutoPaths(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
        m_arm = m_robotContainer.m_arm;
    }

    private Command getDriveCommand(String trajectoryJSONPath) {

        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONPath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            System.out.println("Failed to load path!");
        }
        return m_robotContainer.getDriveCommand(trajectory);
    }

    private Command getShootTop() {
        return m_robotContainer.m_arm.getShootCommand(ArmSubsystem.IntakerMode.SHOOT_TOP);
    }

    private Command getShootMiddle() {
        return m_robotContainer.m_arm.getShootCommand(ArmSubsystem.IntakerMode.SHOOT_MIDDLE);
    }

    private Command getShootBotom() {
        return m_robotContainer.m_arm.getShootCommand(ArmSubsystem.IntakerMode.SHOOT_BOTTOM);
    }

    private Command getToggleIntaker() {
        return m_arm.getToggleIntakerCommand();
    }

    private Command getResetArm() {
        return m_arm.getSetArmPosCommand(ArmSubsystem.ArmPosition.INTAKE);
    }

    private Command getLiftArm() {
        return m_arm.getSetArmPosCommand(ArmSubsystem.ArmPosition.SHOOT);
    }

    public Command getTwoCubeCommand() {
        // Auto: Shoots preloaded cube, goes down, goes to other cube, comes back and shoots (and parks)
        String path = "paths/StraightToCube.wpilib.json";

        return getShootTop()
        .andThen(
            getResetArm(),
            new WaitCommand(0.5),
            getToggleIntaker(),
            Commands.parallel(
                getDriveCommand(path),
                    Commands.sequence(
                    new WaitCommand(3),
                    getToggleIntaker(),
                    getLiftArm(),
                    new WaitCommand(2),
                    getShootMiddle()
                )
            )
        );
    }
}
