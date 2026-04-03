package DroneSimulator;

public class Environment {
	private double width;
	private double depth;
	private double height;
	
	public Environment(double width,double height,double depth) {
		this.width = width;
		this.depth = depth;
		this.height = height;
	}
	public void applyBounds(Drone drone) {
		
		Vector3 pos = drone.getPosition();
		Vector3 vel = drone.getVelocity();
		
		if(pos.x < 0) {
			pos.setX(0);
			vel.setX(-(vel.x));

		}
		else if(pos.x>width) {
			pos.setX(width);
			vel.setX(-(vel.x));

		}
		
		if(pos.y < 0) {
			pos.setY(0);
			vel.setY(-(vel.y));

		}
		else if(pos.y>height) {
			pos.setY(height);
			vel.setY(-(vel.y));

		}
		
		if(pos.z< 0) {
			pos.setZ(0);
			vel.setZ(-(vel.z));

		}
		else if(pos.z>depth) {
			pos.setZ(depth);
			vel.setZ(-(vel.z));

		}
		
//		if(abc) {
//			drone.setPosition(pos);
//			drone.setVelocity(vel);
//		}
	}
	public boolean isInside(Vector3 position) {

	    if (position.x < 0 || position.x > width) {
	        return false;
	    }

	    if (position.y < 0 || position.y > height) {
	        return false;
	    }

	    if (position.z < 0 || position.z > depth) {
	        return false;
	    }

	    return true;
	}
}
