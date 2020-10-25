package frc.robot;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Music {
    Orchestra orchestra;
    int songSelection;
    int timeToPlayLoops;

    ArrayList <TalonFX> instruments = new ArrayList <TalonFX>();
    String[] songs = new String[]{
        "song1.chrp",
        "skechers.chrp",
        "juicy_juicy_midi.chrp",
        "ffMidi.chrp"
    };
    
    
    public void init(){
        instruments.add(new TalonFX(3));
        instruments.add(new TalonFX(1));
        instruments.add( new TalonFX(2));
        //instruments.add(new TalonFX(0));
        orchestra = new Orchestra(instruments);  
        songSelection = 0;
        timeToPlayLoops = 10;
        SmartDashboard.putStringArray("songs", songs);

    }

    public void loadMusic(int offset){
        songSelection += offset;
        
        if(songSelection > songs.length -1){
            songSelection = 0;
        }
        else if(songSelection < 0){
            songSelection = songs.length -1;
        }
        orchestra.loadMusic(songs [songSelection]);
        SmartDashboard.putString("song loaded", songs [songSelection]);
        
            timeToPlayLoops = 10;
    }
    
    public void playMusic(){
        orchestra.play();
        SmartDashboard.putString("is it stopped", "no");
    }

    public void togglePause(){
        if(orchestra.isPlaying()){
            orchestra.pause();
        }
        else
            orchestra.play();
    }

    public void stop(){
        if(orchestra.isPlaying())
            orchestra.stop();
            SmartDashboard.putString("is it stopped", "yes");
    }
    public void periodic(){
        if(timeToPlayLoops>0)
        timeToPlayLoops--;
        else
            playMusic();
    }
    
}


