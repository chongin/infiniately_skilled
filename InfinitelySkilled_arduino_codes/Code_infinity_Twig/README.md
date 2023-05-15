# Infinity Twig

**Inspiration**
https://forum.seeedstudio.com/t/xiao-ble-sense-battery-level-and-charging-status/263248/43?page=2
https://github.com/Swap-File/automaton

I’m controlling it via two wrist bands, both with a XIAO nRF52840 Sense recording motion and sound at 20hz in central mode, reporting via BLE to another Sense in the helmet acting as a peripheral.

The XIAO nRF52840 Sense in the helmet takes the data, processes it, and sends out the results to the LEDs across the helmet, as well as a network of 15 XIAO RP2040s that each control one fin on the helmet. This architecture greatly simplifies the construction.

**Journey**

'''
I tried my hand at writing some c code and found I was trying to overcomplicate the design by creating objects for all the things. Instead, it makes more sense to forget about objects and data encapsulation and instead focus on feature development and classes that when combined implement the features.

This makes the code much more flexible towards change and simplifies the design and implementation IMHO
'''

## TODO

1. ~~Replace all of the code that deals with IMU to use our IMX imu~~
2. ~~Add in the Magnetometer code~~
3. Verify measurements [ax,ay,az,gx,gy,gz,mx,my,mz]
4. Verify quaternion calculations
5. Adjust measurements due to device orientation / determine process for calibration
6. Implement mobile unity app to record session data over BLE services and to display device orientation in earth frame reference / absolute orientation
7. Implement wifi version to send data to a local server over UDP / elixir
8. Create uwb.cpp comms and wire up the Hub and the Anchor projects so that we can get positional data as well as receive and send data to local server for in-arena solution!
9. Train MLC (machine learning core) decision trees and FSM (finite state machines) so that we can detect and interrupt the different hockey stick movements and transitions and store the counts in the hardware registers ;) - i.e. add in event.cpp handling
10. Start building out unity game engine (This is when we can approach hockey canada?)

No need for a personal trainer!

- Stickhandling challenges (personal/team)
- Shooting challenges (personal / team)
- Get those reps in! Alerts - (discipline > motivation)
- Show skill progression chart - where the player is in the skill acquisition chart by how much they have improved - ability to scientifically show where they started and where they are and where they need to get to be the best!
- CPN stickhandling
  - Teach each skill in succession and then combine the skills
- Head to head compete (mirror mode - record and compete)
- Record the best players skills and use that as a baseline (hands like McDavid / quick release like Connor Bedard / etc... / etc...)
- If it's sponsored by Hockey Canada, maybe we can get the superstars involved somehow, especially on the concussion prevention front!!! See if we can get a few of these players to tweet about it (Eric Lindros / etc...)
- Player, parent, coach testimonials!!!!
- Team discounts / league / association discounts

11. Start building out game statistics / manager for clubs (OHL/University)

- Each player has their own personal goals but coaches can see progression on the individual player level.

Note: In order to get this into the hands of as many hockey players as possible, we need it to be game first with leaderboards. Then we need to sell it to hockey canada as a licensed program. This would really help us create the next level of highly skilled players who have higher hockey IQ because they have better hands and vision having learned to stickhandle with their peripheral vision with a proper head up/chin up position.

Note: We absolutely must get the OHL Sudbury Wolves team on board with this solution and the stats! It's our home turf and backyard. We can get the Lady Wolves and Sudbury Minor Hockey as well as Bishop on board next!

Infinitely Skilled Inc.
Next Level Hockey Player Skill Development Program

- Infinity Twig / Twig Hero
- Infinity Skates - I think we could use the same sensors on skates! We would simply integrate acceleration over time to be able to show speed as well as arcs! Help players with their weight distribution and angle of skate blades!
- Infinity Stats
