package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUMecDT extends MecDT {
    private IMU imu;
    private double forwards = 0;

    void setIMU(IMU imu){
        this.imu = imu;
    }
    IMU getIMU(){
        return imu;
    }

    void initIMU(
            RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection,
            RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection
    ){
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    void setForwards(double direction){
        forwards = direction;
    }
    void setForwards(){
        setForwards(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    double getDirectionRaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    double getDirection(){
        return getDirectionRaw() - forwards;
    }

    void absoluteDirectionalPow(double forward, double side, double rot){
        absoluteDirectionalPow(getDirection(), forward, side, rot);
    }
}
