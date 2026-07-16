⚖️ License & Usage Terms
Copyright (c) 2026 [S-DiY channal/ truongvansu91]. All Rights Reserved.
Email:Sdiychannel01@gmail.com

This software and its related documentation are provided under the following restrictive terms:

Personal Use Only: You are permitted to use, modify, and flash this code for personal, non-commercial hobbyist projects.

Non-Commercial Clause: You may not sell this code, use it in a commercial product, or use it for any profit-making activities without explicit written consent.

Attribution Requirement: If you feature this project or code in creative content (e.g., YouTube videos, TikTok, social media, or blogs),
 you must provide clear credit to the author and link back to the original source.

Note: Use only with Digital servos. Using Analog servos will cause them to fail quickly 

This is the Triangle wave version. The wings flap fast throughout the stroke and reverse direction sharply at the limits. The advantage is a higher flapping speed and more power, 

but the servos will run hotter than in the Sine wave version

Max Flap Angle	flapAmplitude	40 - 85	Sets the peak travel (degrees) from the 90° center point. Values above 80 may strain mechanical linkages.

Pulse Width	attach(pin, 500, 2500)	500 - 2500 us	Extends physical servo travel. Verify that your servos support 2500us to avoid motor burnout.

Flapping Speed	speedFactor	50 - 150	Mapped to the Throttle channel. Defines the phase increment per loop (Sine/Triangle wave frequency).

