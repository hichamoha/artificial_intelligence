package models;

//import viewer.RobotLocalizationViewer;
import java.util.*;

public class RobotSim {
	/*
	 *  ********************* YOUR TASK ******************
	 *  
	 *  Implement the robot simulator:
	 *  - the robot should be able to move one step according to the motion model 
	 *  i.e., produce a new pose based on the old one
	 *  - the simulation should also contain a sensor reading method, 
	 *  that generates a sensor reading based on the current (true)
     *  state of the system and the sensor model
	 */
    	                                        // nextPosition()
	                                            // getCurrentReading()
    TransitionMatrix tm;
    SensorMatrices sm;
    StateModel stateModel;

    // number of rows, columns and headings
    private int rows, cols, head;
	
    // the current true state and the subsequent sensor reading
    private int trueState, sense;

    private static final Random rand = new Random();
    
    public RobotSim(StateModel stateModel){
    	this.stateModel = stateModel;
    	int[] help = stateModel.getDimensions();
		
    	rows = help[0];
    	cols = help[1];
		head = help[2];

    	tm = new TransitionMatrix(stateModel);
    	sm = new SensorMatrices(stateModel);
		
    	// initialize true state randomly
	

	//trueState = 0;
	//sense = -1;
    }	
	
     // XXXXXXXXXXX TO DO XXXXXXXXXXXXXXXXXXXXXXXXX    
    // nextPosition(){}
    // getCurrentReading(){}


}
