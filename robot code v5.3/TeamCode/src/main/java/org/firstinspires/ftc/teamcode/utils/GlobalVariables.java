package org.firstinspires.ftc.teamcode.utils;

public class GlobalVariables {
     private static double endAutoRobotAngle;
     public static int endAutoRampEncoder;
     public static int endAutoArmEncoder;
     public static void reset (){
         endAutoRobotAngle = 0;
         endAutoRampEncoder = 0;
         endAutoArmEncoder = 0;
     }

    public static void setEndAutoRobotAngle(double endAutoRobotAngle) {
        while (endAutoRobotAngle > 180) endAutoRobotAngle -= 360;
        while (endAutoRobotAngle <= -179)  endAutoRobotAngle += 360;

        GlobalVariables.endAutoRobotAngle = endAutoRobotAngle;
    }

    public static double getEndAutoRobotAngle() {
        return endAutoRobotAngle;
    }
}
