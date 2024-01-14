package frc.robot.commands;

import frc.robot.Profile;
import frc.robot.Infrastructure;

public abstract class Command {
    private final Infrastructure infrastructure = Infrastructure.getInstance();
    private Command endCommand;

    private Profile.Button button;

    protected void onInit() {

    }

    public void periodic() {

    }

    protected void onEnd() {

    }

    public void init() {
        onInit();
    }

    public final void end() {
        infrastructure.removeCommand(this);

        if (endCommand != null) {
            infrastructure.registerCommand(endCommand);
        }

        onEnd();
    }

    public void addEndCommand(Command nextCommand) {
        endCommand = nextCommand;
    }

    public void setRunButton(Profile.Button button) {
        this.button = button;
    }

    public void endWithoutNextCommand() {
        infrastructure.removeCommand(this);

        onEnd();
    }

    public Profile.Button getRunButton() {
        return button;
    }
}
