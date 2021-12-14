package frc;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;

/** Add your docs here. */
public class JoystickNT
{
    private String Name;
    private NetworkTableInstance inst;
    private NetworkTable table;
    private NetworkTableEntry[] axes;
    private NetworkTableEntry[] buttons;


    private String[] axisNames = { "Axis-0", "Axis-1", "Axis-2", "Axis-3", "Axis-4", "Axis-5", "Slider-0", "Slider-1" };
    private String[] buttonNames = { "Button-0", "Button-1", "Button-2", "Button-3", "Button-4", "Button-5", "Button-6", "Button-7" };

    public JoystickNT(String name)
    {
        Name = name;

        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("SmartDashboard");
        axes = new NetworkTableEntry[axisNames.length];
        buttons = new NetworkTableEntry[buttonNames.length];

        // for (int i=0; i<axisNames.length; i++)
        //     axes[i] = table.getEntry(axisNames[i]);

        // for (int i=0; i<buttonNames.length; i++)
        //     buttons[i] = table.getEntry(buttonNames[i]);
    }

    public double getRawAxis(int axis)
    {
        if (axis < 0 || axis >= axisNames.length)
            return 0.0;

        // Lazy initialization of axes table

        if (/*axes == null || axis >= axes.length || */axes[axis] == null)
        {
            axes[axis] = table.getEntry(Name + "/" + axisNames[axis]); 
        }

        // If provider not supplying yet....

        if (axes[axis] == null)
            return 0.0;

        double rawValue = axes[axis].getDouble(27.0);
        return (rawValue - 16384) / 16384;
    }

    public boolean getRawButton(int button)
    {
        if (button < 0 || button >= buttonNames.length)
            return false;

        // Lazy initialization of axes table

        if (/*buttons == null || button >= buttons.length || */buttons[button] == null)
        {
            buttons[button] = table.getEntry(Name + "/" + buttonNames[button]); 
        }

        // If provider not supplying yet....

        if (buttons[button] == null)
            return false;

        return buttons[button].getBoolean(false);
    }

}
