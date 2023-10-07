package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Hardware {
    private IMUMecDT drivetrain = new IMUMecDT();

    void init(HardwareMap hardwareMap){
        drivetrain.setMotors(
                hardwareMap.get(DcMotor.class, "leftfront_drive"),
                hardwareMap.get(DcMotor.class, "rightfront_drive"),
                hardwareMap.get(DcMotor.class, "leftback_drive"),
                hardwareMap.get(DcMotor.class, "rightback_drive")
        );
        drivetrain.setDirections(
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        );

        drivetrain.setIMU(
                hardwareMap.get(IMU.class, "imu")
        );
        drivetrain.initIMU(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
    }

    IMUMecDT getDrivetrain(){
        return drivetrain;
    }
}
