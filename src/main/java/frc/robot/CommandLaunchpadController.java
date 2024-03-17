// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class CommandLaunchpadController extends CommandJoystick {

    private static final int ARM_UP = 8;
    private static final int ARM_DOWN = 9;
    private static final int INTAKE_IN = 10;
    private static final int SHOOTER_OUT = 11;
    private static final int SAFETY = 4;
    private static final int CLIMB_UP = 7;
    private static final int CLIMB_DOWN = 6;
    private static final int MISC_BLUE = 5;

    public CommandLaunchpadController(int port) {
        super(port);
    }

    public Trigger armUp() {
        return button(ARM_UP);
    }

    public Trigger armDown() {
        return button(ARM_DOWN);
    }

    public Trigger intakeIn() {
        return button(INTAKE_IN);
    }

    public Trigger shooterOut() {
        return button(SHOOTER_OUT);
    }

    public Trigger safety() {
        return button(SAFETY);
    }

    public Trigger climbUp() {
        return button(CLIMB_UP);
    }

    public Trigger climbDown() {
        return button(CLIMB_DOWN);
    }

    public Trigger miscBlue() {
        return button(MISC_BLUE);
    }

}
