package org.firstinspires.ftc.teamcode.API.HW;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import lombok.Getter;
import lombok.Setter;

public class SmartColorSensor {
    @Getter NormalizedColorSensor sensor;

    public SmartColorSensor(NormalizedColorSensor sensor) {
        this.sensor = sensor;
    }

    // Color sensor config
    @Getter @Setter double redFudge, greenFudge, blueFudge = 1;

    public NormalizedRGBA getNormalizedColors() {
        return sensor.getNormalizedColors();
    }
}
