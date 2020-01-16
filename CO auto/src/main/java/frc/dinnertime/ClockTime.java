/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.dinnertime;

/**
 * Add your docs here.
 */
public class ClockTime {
    int hour = 0;
    int minute = 0;
    int second = 0;
    public ClockTime(int _hour, int _minute, int _second){
        hour = _hour;
        minute = _minute;
        second = _second;
    }

    public String giveMeTime(){
        return hour + ":" + minute + ":" + second;
    }
}
