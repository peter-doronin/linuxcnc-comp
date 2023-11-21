double filter(double input, double time_in, double f_per)
	{
		#define RING_SIZE 1000
		int time = (int)(fabs(time_in) / f_per); 
		if(time == 0) return input;
		if(time > RING_SIZE) time = RING_SIZE;
		
		static unsigned int ring_index;
		static double ring_buf[RING_SIZE];
		ring_index++;
		ring_index %= time;
		ring_buf[ring_index] = input;
		double f_out;
		for(int i = 0; i < time; i++)
		{
			f_out += ring_buf[i];
		}
		f_out /= time;
		return f_out;
	}
