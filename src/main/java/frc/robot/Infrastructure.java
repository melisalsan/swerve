package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.commands.Command;
import frc.robot.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class Infrastructure extends Subsystem {
    private static Infrastructure instance;

    private final PowerDistribution pdh = new PowerDistribution();

    public boolean enabled = false;

    public static Infrastructure getInstance() {
        if (instance == null) {
            instance = new Infrastructure();
        }
        return instance;
    }

    private Infrastructure() {

    }

    private final List<Command> commandQueue = new ArrayList<>();

    public boolean isCommandInQueue(Class<? extends Command> commandType) {
        if (commandType == null) return false;
        
        for (Command command : commandQueue) {
            if (command.getClass().equals(commandType)) return true;
        }
        return false;
    }

    public void registerCommand(Command command) {
        if (command == null) {
            return;
        }

        if (isCommandInQueue(command.getClass())) return;
        this.commandQueue.add(command);
        command.init();
    }

    public void registerCommandWhileHeld(Command command, Profile.Button button) {
        if (command == null || !button.getButtonPressed()) {
            return;
        }

        command.setRunButton(button);
        registerCommand(command);
    }

    public void removeCommand(Command command) {
        commandQueue.remove(command);
    }
    
    public void clear() {
        for (int i = 0; i < commandQueue.size(); i++) {
            commandQueue.get(i).endWithoutNextCommand();
        }
    }

    public double getVoltage() {
        return pdh.getVoltage();
    }

    @Override
    public void robotPeriodic() {
        if (enabled) {
            for (int i = 0; i < commandQueue.size(); i++) {
                Command command = commandQueue.get(i);
                if (command.getRunButton() == null) {
                    command.periodic();
                } else if (command.getRunButton().getButtonState()) {
                    command.periodic();
                } else {
                    command.end();
                }
            }
        }
    }

    @Override
    public void robotInit() {
        clear();
    }

    @Override
    public void enabledInit() {
        clear();
    }

    @Override
    public void teleopInit() {
        clear();
    }

    @Override
    public void disabledInit() {
        for (int i = 0; i < commandQueue.size(); i++) {
            commandQueue.get(i).end();
        }

        enabled = false;
    }

    @Override
    public void testInit() {
        clear();
    }


}
