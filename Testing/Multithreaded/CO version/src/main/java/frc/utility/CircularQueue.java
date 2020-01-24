// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility;
import frc.utility.math.InterpolablePair;

/**
 * NOT thread safe. This class assumes the data added is sorted! It has
 * O(1)insertion (only able to insert at end) and O(n log n) search
 */
public class CircularQueue<T> {
	
	private InterpolablePair<T>[] queue;
	private long back;
	public final int size;

	@SuppressWarnings("unchecked")
	public CircularQueue(int size) {
		queue = new InterpolablePair[size];
		back = 0;
		this.size = size;
	}
	/**
	 *
	 * @param t
	 *            Add new a new value to the end of the queue
	 */
	public void add(InterpolablePair<T> t) {
		queue[(int) back % size] = t;
		back++;
	}

	/**
	 * Get InterpolableValue<T> from the queue that is a specified distance from
	 * the back
	 *
	 * @param position
	 *            Distance from back of queue
	 * @return InterpolableValue<T> from position in argument
	 */
	public InterpolablePair<T> getFromQueue(int position) {
		position %= size;
		return queue[(int) (back - position - 1) % size];
	}

	public int getLength() {
		return queue.length;
	}
}