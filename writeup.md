## Parameter initialization

Initial parameters were chosen by experimentation. Firstly, we need to choose Kp
parameter. The car should have oscillating behavior. I chose Kp = 0.05. 
Then we need to eliminate oscillation. We can do that by changing Kd parameter.
I chose Kd = 1.5. And finally we can reduce car's bias by choosing Ki = 0.0005.

After we chose initial values, we can optimize these parameters by using twiddle 
algorithm. After applying this algorithms I got following values: 
Kp = 0.232, Ki = 0.0001, Kd = 1.58.