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
public class FoodItem {
    MUNCHY munchy;
    CRUNCHY crunchy;
    SLIMY slimy;
    GRIMY grimy;
    public FoodItem(MUNCHY m, CRUNCHY c, SLIMY s, GRIMY g){
        munchy = m;
        crunchy = c;
        slimy = s;
        grimy = g;
    }
    public enum MUNCHY{
        DIRT,
        POOPOO,
        HAMBURGER,
        GUMMY
    }
    public enum CRUNCHY{
        SAND,
        SOLIDIFIED,
        SANDWICH,
        GOLDFISH
    }
    public enum SLIMY{
        MORGANFREEMAN
    }
    public enum GRIMY{
        N
    }
}
