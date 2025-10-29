package org.firstinspires.ftc.teamcode.robotCode2025.imu;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUClass {

    public IMU imu ;

    public void imuInit(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class,"imu");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );

        imu.initialize(new IMU.Parameters(revOrientation));

    }

    public double imuGetYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
