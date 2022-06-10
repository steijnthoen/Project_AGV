// moet refrence naar gyroscope.c
// moet refrence naar functie die agv laat draaien


int gyroscope_refrence; // waarde van de gyroscope wanneer hij begint met draaien

void mass_update_gyroscope()
{
	int i;
	for(i = 0; i<15; i++)
	{
		gyroscope_update();
	}
}



void start_gyroscope()
{
	gyroscope_init();
	mass_update_gyroscope();
	gyroscope_refrence = gyroscope_get_rotation_Z();
}



// uitgaande dat gyroscoop optelt wanneer draait met klok mee
void bocht_rechts(int graden_bocht)
{
	mass_update_gyroscope();
	gyroscope_refrence = gyroscope_get_rotation_Z();

	int current_rotation = 0;

	// start bocht rechts

	while(current_rotation >= graden_bocht)
	{
		current_rotation = (gyroscope_get_rotation_Z() - gyroscope_refrence);
		gyroscope_update();
	}

	// stop motoren
}



// uitgaande dat gyroscoop aftrekt wanneer draait tegen de klok in
void bocht_links(int graden_bocht)
{
	mass_update_gyroscope();
	gyroscope_refrence = gyroscope_get_rotation_Z();

	int current_rotation = 0;

	// start bocht links
	while(current_rotation <= -graden_bocht)
	{
		current_rotation = (gyroscope_get_rotation_Z() - gyroscope_refrence);
		if(current_rotation > 0)
		{
			current_rotation -= 360;
		}

		gyroscope_update();
	}

	// stop motoren
}

