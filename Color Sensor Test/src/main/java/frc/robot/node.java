package frc.robot;
import edu.wpi.first.wpilibj.util.Color;
class Node 
 {
    Color color;
    int value;
    Node nextNode;
 
    public Node(int value) {
        this.value = value;
    }

    public Node(Color myColor)
    {
      this.color = myColor;
    }
}