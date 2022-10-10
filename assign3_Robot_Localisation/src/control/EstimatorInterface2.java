package control;

import models.*;

/*
 * This interface provides the graphics class (RobotLocalizationViewer) with the 
 * necessary access points to the data it wants to display.
 * If you want to use the viewer, have some class implement this interface, which 
 * you could see as a wrapper to your internal model for the transition and sensor 
 * matrices as well as the probability distribution over the possible positions
 * 
 * The viewer assumes a MxN-grid and in each field (m,n) the robot can have four 
 * different headings. Your implementation of this interface needs hence to provide 
 * the requested data in relation to this assumption!
 */
public interface EstimatorInterface2 {

	/*
	 * return the number of assumed rows, columns and possible headings for the grid
	 * a number of four headings makes obviously most sense... the viewer can handle 
	 * four headings at maximum in a useful way.
	 */
	public int getNumRows();
	public int getNumCols();
	public int getNumHead();
	
	/*
	 * should trigger one step of the estimation, i.e., true position, sensor reading and 
	 * the probability distribution for the position estimate should be updated and filled in.
	 * 
	 * Return true if the sensor gave a valid reading.
	 */
	public boolean update( int[] trueState, int[] sensedPos);
	
	/*
	 * returns the currently known true position i.e., after one simulation step
	 * of the robot as (x,y)-pair.
	 */
	public int[] getCurrentTrueState();
	
	/*
	 * returns the currently available sensor reading obtained for the true position 
	 * after the simulation step 
	 * returns null if the reading was "nothing" (whatever that stands for in your model)
	 */
	public int[] getCurrentReading();
	
	/*
	 * returns the currently estimated (summed) probability for the robot to be in position
	 * (x,y) in the grid. The different headings are not considered, as it makes the 
	 * view somewhat unclear.
	 */
    public double[] getCurrentProbDist();

	/*
	 * returns the collection of sensor matrices O_r containing the probabilities to get reading r corresponding 
	 * to position (rX, rY) when actually in position (x, y)
	 */
	
    public SensorMatrices getObservationMatrices();

	/*
	 * returns the transition matrix T, in which each entry (Tij) contains the probability to go from pose 
	 * i = (x, y, h) to pose j = (nX, nY, nH)
	 */	
	
    public TransitionMatrix getTMatrix();
    
    // - - - - - - ------------  - -------
    
    //    public double getTProb( int x, int y, int h, int nX, int nY, int nH); // XXXXXX

    //    public double getOrXY( int rX, int rY, int x, int y, int h); // XXXXX

    //    public double getCurrentProb( int x, int y);  // XXXXX

}
