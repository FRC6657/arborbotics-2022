/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj2.command;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A command that runs a given runnable when it is initalized, and another runnable when it ends.
 * Useful for default commands of a subsystem
 **/
public class RunEndCommand extends CommandBase {
    protected final Runnable m_onExecute;
    protected final Runnable m_onEnd;

    /**
     * Creates a new RunEndCommand.  Will run the given runnables when the command starts and when
     * it ends.
     *
     * @param onExecute       the Runnable to run on command execute
     * @param onEnd        the Runnable to run on command end
     * @param requirements the subsystems required by this command
     */
    public RunEndCommand(Runnable onExecute, Runnable onEnd, SubsystemBase... requirements) {
        m_onExecute = requireNonNullParam(onExecute, "onInit", "RunEndCommand");
        m_onEnd = requireNonNullParam(onEnd, "onEnd", "RunEndCommand");

        addRequirements(requirements);
    }

    @Override
    public void execute() {
        m_onExecute.run();
    }

    @Override
    public void end(boolean interrupted) {
        m_onEnd.run();
    }
}