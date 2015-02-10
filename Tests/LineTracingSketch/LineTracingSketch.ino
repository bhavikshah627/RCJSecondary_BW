void setup()
{
	//#include
}

void loop()
{
	//getlight() //unused function
	/*			  -----
				  | 8 |
				  -----
	 /-----------------------------\
	| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
	 \-----------------------------/
	 0 - 2 Left
	 3 - 4 Unused
	 5 - 7 Right
	*/
	/*
	if right sees green					//intersection
		right 0
		left 100
	else if right sees not green
		if left sees green				//intersection
			right 100
			left 0
		else if left sees not green
			if right sees black
				if left sees black
					right 100
					left 100
				else if left sees white
					right 0
					left 50
			else if right sees white
				if left sees black
					right 50
					left 0
				else if left sees white
					right 100
					left 100
	*/
}
