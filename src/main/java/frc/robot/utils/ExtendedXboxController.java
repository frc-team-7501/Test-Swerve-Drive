// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ExtendedXboxController extends XboxController {
    private JoystickButton k_b_LeftBumper = getButton(Button.kLeftBumper);
    private JoystickButton k_b_RightBumper = getButton(Button.kRightBumper);
    private JoystickButton k_b_LeftStick = getButton(Button.kLeftStick);
    private JoystickButton k_b_RightStick = getButton(Button.kRightStick);
    private JoystickButton k_b_A = getButton(Button.kA);
    private JoystickButton k_b_B = getButton(Button.kB);
    private JoystickButton k_b_X = getButton(Button.kX);
    private JoystickButton k_b_Y = getButton(Button.kY);
    private JoystickButton k_b_Back = getButton(Button.kBack);
    private JoystickButton k_b_Start = getButton(Button.kStart);

    public ExtendedXboxController(int port) {
        super(port);
    }

    public JoystickButton getButton(int button) {
        return new JoystickButton(this, button);
    }

    public JoystickButton getButton(Button button) {
        return new JoystickButton(this, button.value);
    }

    public JoystickButton b_LeftBumper() { return k_b_LeftBumper; }
    public JoystickButton b_RightBumper() { return k_b_RightBumper; }
    public JoystickButton b_LeftStick() { return k_b_LeftStick; }
    public JoystickButton b_RightStick() { return k_b_RightStick; }
    public JoystickButton b_A() { return k_b_A; }
    public JoystickButton b_B() { return k_b_B; }
    public JoystickButton b_X() { return k_b_X; }
    public JoystickButton b_Y() { return k_b_Y; }
    public JoystickButton b_Back() { return k_b_Back; }
    public JoystickButton b_Start() { return k_b_Start; }
}
