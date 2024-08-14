//Attempt at debouncing buttons efficiently and with minimal latency at the positive edge
//This assumes "updateButton" is called at a rate signifficantly (at least an order of magnitude) above the haptic rate
//Code by Kassen Oud. Should you run into this and find it useful; you can use it under the GPL2
//Shoutouts to Rob Bothof for the "using a byte as a history of states" trick

//our struct
struct Button
{
	bool state;					//current state of the button as we see it
	unsigned char history;		//history of the last 8 measured states
};

//takes a Button struct and the currently measured value of the relevant pin
//returns true if the state changed at which point "my_button.state" can be used by your program to get the new state
bool updateButton( struct Button *but, bool in)
{
	but->history = but->history << 1;							//push back history
	but->history = but->history | (unsigned char)in;			//store input in most recent history location
	if (but->state)
	{
		if (!but->history)										//last 8 values were all "off"
		{
			but->state = false;
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		if (in)													//we assume buttons do not accidentally turn on
		{
			but->state = true;
			return true;
		}
		else
		{
			return false;;
		}
	}
}
