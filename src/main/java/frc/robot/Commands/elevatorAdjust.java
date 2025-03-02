package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.ElvManipSubsystem;

public class elevatorAdjust extends Command{
    ElvManipSubsystem manipulator;
    CommandXboxController controller;

    public elevatorAdjust(ElvManipSubsystem manipulator, CommandXboxController controller2) {
        this.manipulator = manipulator;
        this.addRequirements(this.manipulator);
        this.controller = controller2;
    }

    @Override
    public void execute() {
        double axis = controller.getLeftY();
        System.out.println(axis);
        manipulator.adjustHeight(axis * 2);
    }
}
