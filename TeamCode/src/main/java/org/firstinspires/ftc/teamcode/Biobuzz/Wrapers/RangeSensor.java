package org.firstinspires.ftc.teamcode.Biobuzz.Wrapers;


import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cDeviceType
@DeviceProperties(
        xmlTag = "RangeSensor",
        name = "Range Sensor - REV 3"
)
public class RangeSensor extends Rev2mDistanceSensor implements HardwareDevice{

    private double freq = 0;
    private double lastDist = 0;
    private double NewResultContribution = 1;
    private double timePassed = 0;
    public RangeSensor(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.timePassed = System.currentTimeMillis();
        freq = 50;
    }

    public void changeFrequency(double freq){
        this.freq = freq;
    }
    public void changeNextResultContribution(double contribution){
        this.NewResultContribution = contribution;
    }
    public double getDist(){
        double dist = lastDist;
        if(System.currentTimeMillis() - timePassed > freq) {
            dist = NewResultContribution * this.getDistance(DistanceUnit.CM) + (1 - NewResultContribution) * lastDist;
            timePassed = System.currentTimeMillis();
            lastDist = dist;
        }
        return dist;
    }

}
