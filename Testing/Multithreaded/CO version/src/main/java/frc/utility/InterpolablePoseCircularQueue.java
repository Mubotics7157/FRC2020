package frc.utility;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class InterpolablePoseCircularQueue extends CircularQueue<Pose2d> {
    public InterpolablePoseCircularQueue(int size) {
        super(size);
    }

    
	/**
	 * Get T with given key. If an exact match isn't found it will interpolate
	 * the value always. Keys outside the range will return the front or end of
	 * the queue.
	 *
	 * @param key
	 *            Key of wanted T
	 * @return Matching interpolated T from key in argument
	 */
	public Pose2d getInterpolatedPose(long key) {
		int low = 0;
		int high = getLength() - 1;
		while (low <= high) {
			int mid = (low + high) / 2;
			double midVal = getFromQueue(mid).getKey();
			if (midVal < key) {
				low = mid + 1;
			} else if (midVal > key) {
				high = mid - 1;
			} else {
				return getFromQueue(mid).getValue();
			}
		}
		double difference = key - getFromQueue(low).getKey();
		double total = getFromQueue(high).getKey() - getFromQueue(low).getKey();
		return interpolate(getFromQueue(low).getValue(), getFromQueue(high).getValue(), difference / total);
    }
    
	public Pose2d interpolate(Pose2d initial, Pose2d other, double percentage) {
		Translation2d delta = new Translation2d(initial.getTranslation().getX() - other.getTranslation().getX(), initial.getTranslation().getY() - other.getTranslation().getY());
		Rotation2d deltaR = initial.getRotation().minus(other.getRotation());
		return new Pose2d(
			new Translation2d(initial.getTranslation().getX() + delta.getX() * percentage, 
							  initial.getTranslation().getY() + delta.getY() * percentage),
							  initial.getRotation().plus(deltaR.times(percentage)));
	}
}