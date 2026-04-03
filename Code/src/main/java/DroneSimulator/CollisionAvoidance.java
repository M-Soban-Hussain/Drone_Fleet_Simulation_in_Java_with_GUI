package DroneSimulator;

import java.util.List;

public class CollisionAvoidance {

    private double Krep; 
    private double Dmin;    
    private int col;
    
    
    public CollisionAvoidance(double Krep, double Dmin) {
        this.Krep = Krep;
        this.Dmin = Dmin;
        this.col = 0;
    }

    public Vector3 computeRepulsiveForce(Drone self, List<Drone> neighbors) {

        Vector3 totalRepulsion = Vector3.zero();
        Vector3 pi = self.getPosition();

        for (Drone neighbor : neighbors) {
            if (neighbor.getId() == self.getId()) {
                continue;
            }

            Vector3 pj = neighbor.getPosition();

            Vector3 diff = pi.subtract(pj);
            double distance = diff.magnitude();


            if (distance > 0 && distance < Dmin) {
            	
            	setCol(countCollisions(neighbors));
                Vector3 force =diff.multiply(this.Krep / (distance * distance));

                totalRepulsion = totalRepulsion.add(force);
            }
        }

        return totalRepulsion;
    }
    
    public int countCollisions(List<Drone> drones) {
        int collisionCount = 0;
        
        for (int i = 0; i < drones.size(); i++) {
            for (int j = i + 1; j < drones.size(); j++) {
                Vector3 pi = drones.get(i).getPosition();
                Vector3 pj = drones.get(j).getPosition();
                
                if (pi.distance(pj) < Dmin) {
                    collisionCount++;
                }
            }
        }
        
        return collisionCount;
    }

	public int getCol() {
		return col;
	}

	public void setCol(int col) {
		this.col = col;
	}
}
