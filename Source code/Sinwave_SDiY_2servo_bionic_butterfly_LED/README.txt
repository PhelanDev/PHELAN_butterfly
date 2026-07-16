⚖️ License & Usage Terms
Copyright (c) 2026 [S-DiY channal/ truongvansu91]. All Rights Reserved.
Email:Sdiychannel01@gmail.com

This software and its related documentation are provided under the following restrictive terms:

Personal Use Only: You are permitted to use, modify, and flash this code for personal, non-commercial hobbyist projects.

Non-Commercial Clause: You may not sell this code, use it in a commercial product, or use it for any profit-making activities without explicit written consent.

Attribution Requirement: If you feature this project or code in creative content (e.g., YouTube videos, TikTok, social media, or blogs),

Note: Use only with Digital servos. Using Analog servos will cause them to fail quickly 

you must provide clear credit to the author and link back to the original source.

This is the Sine wave version. The mechanism makes the servos move fast in the middle and slow down at the edges.
 
The advantage is a more lifelike flapping motion that saves energy, though the flapping speed and power are weaker than the Triangle wave version.

Max Flap Angle	flapAmplitude	40 - 85	Sets the peak travel (degrees) from the 90° center point. Values above 80 may strain mechanical linkages.

Pulse Width	attach(pin, 500, 2500)	500 - 2500 us	Extends physical servo travel. Verify that your servos support 2500us to avoid motor burnout.

Flapping Speed	speedFactor	50 - 150	Mapped to the Throttle channel. Defines the phase increment per loop (Sine/Triangle wave frequency).

