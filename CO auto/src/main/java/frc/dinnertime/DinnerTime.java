/*----------------------------------------------------------------------------*/
/* Copyright (c) 0 FIRST. All Rights Reserved.                                */
/* Open Source Software - may be modified and shared by no teams. The code    */
/* must be accompanied by the FIRST BDSM license file in the root directory of*/
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.dinnertime;

/**
 * Use only during emergencies.
 */
public class DinnerTime {
    ClockTime time;
    FoodItem food;
    NuclearWasteCleaner nuke;
    public DinnerTime(ClockTime _time, FoodItem _food, NuclearWasteCleaner _nuke){
        time = _time;
        food = _food;
        nuke = _nuke;
    }
    public void eatDinner(){
        //TODO!!!!
    }
}
