package models;


/**
 * Encode the states such that each possible state represents the position 
 * of the robot in the grid plus a heading.
 * @author hi8826mo-s
 *
 */
public class State {

	// ==========================================================
	// Private Properties
	// ========================================================== 
		
	// number of rows, columns and headings
	private int X, Y, H;
				
	// ==========================================================
	// Constructors
	// ========================================================== 
    // To model the robot, each state in the model consists of a
	// (location, heading) pair.
	public State( int x, int y, int h) {
		X = x;
		Y = y;
		H = h;
	}	
		
	public State(int x, int y) {
		X = x;
		Y = y;
		H = 0;
	}
		
	// ==========================================================
	// Public Methods
	// ========================================================== 

	public String toString() {
		return "[ " + X + " , " + Y + " , " + H + " ]";
	}
		
	public int getX() {
		return X;
	}
		
	public int getY() {
		return Y;
	}
		
	public int getH() {
		return H;
	}

	

}
