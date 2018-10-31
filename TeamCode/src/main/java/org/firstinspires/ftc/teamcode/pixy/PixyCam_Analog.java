package org.firstinspires.ftc.teamcode.pixy;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Use this one when setting the PixyMon to "analog/digital x"

public class PixyCam_Analog  {
    private AnalogInput analogIn_pixy;

    public PixyCam_Analog(HardwareMap hardwareMap) {
        analogIn_pixy       = hardwareMap.analogInput.get("pixy_analog");       // if it doesn't work, try the above
    }

    // reading from pin 8 (Analog out) from Pixy
    public double getAnalogRead() {   // from 0 to 3.3V where 1.15V=middle, higher means object is to the right
        return  analogIn_pixy.getVoltage();
    }
}