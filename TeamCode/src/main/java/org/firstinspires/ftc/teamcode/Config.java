package org.firstinspires.ftc.teamcode;

public class Config {
    Hardware hardware = new Hardware();

    //Constructor
    public Config() {}

    //Drivertrain
    double wheelCPMM = (((hardware.hdHexCPR * hardware.ultraFiveToOne * hardware.ultraFourToOne) / hardware.mecanumWheelCircumfrence));
    double wheelBaseWidth = 385.0; //mm
    double wheelBaseLength = 305.0; //mm
    double wheelBaseDiameter = Math.sqrt(Math.pow(wheelBaseWidth, 2.0) + Math.pow(wheelBaseLength, 2.0)); //mm
    double wheelBaseCircumference = Math.PI * wheelBaseDiameter; //mm

    //General
    double stickDeadzone = 0.1;
    long loopTime = 20; //ms
    long loopsPerSecond = 1000/loopTime;

    //Drive
    double driveSpeedMultiplier = 2;
    double slowDriveSpeedMultiplier = 0.2;
    double acceleration = 0.05;

    //Pivot
    double pivotSpeedMultiplier = 1;
    double slowPivotSpeedMultiplier = 0.5;

    //Arm
    double armMinimumPosition = 0;
    double armMaximumPosition = 1000;
    double armStep = 5;

    // Slide

    double slideMinimumPosition = 0;
    double slideMaximumPosition = 1200;

    double slideMaximumPositionHigh = 2500;

    double slideSecondPosition = 1050; // robot is 40 inches

    double slideHighPosition = 4000; // max slide

    double slideStep = 10;




}