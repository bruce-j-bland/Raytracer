# Raytracer
A software raytracer built for the class Global Illuminations.

The raytracer implements a full suite of features: variant illumination models, reflection, refraction, texture mapping, and a handful of tone reproduction models.

The raytracing process is multithreaded, and is capable of edge-aware supersampling.

The SDL2 library is used for displaying the generated pixel maps.

Running the command line program allows you to select between one of three scenes: areproduction of the first ray traced image by Whitted, a rendering of the Stanford Bunny, and working, real-time software raytraced implementation of the classic board of ThunkTS (https://github.com/bruce-j-bland/ThunkTS).

The Standford Bunny implements a simple ply parser and uses a K-D Tree spacial data structure to handle the computationaly complex object.

The ThunkTS implementation is procedurally generated from each boardstate, and makes use of a bespoke spatial data structure. The image is object-aware, and will selectively rerender individual pixels based on game-state updates. Some animation is implemented when a placed wall or a player falls into the center hole.

The UI is entirely mouse based. Click on a tile once to select it, and a second time to confirm. The color of the selection area matches the color of the current player. During the drop phase, any tile without a player or wall in it is a legal selection. During the movement phase, only tiles on the edge of the board are legal.

A more detailed explanation of gameplay and rules can be found in the ThunkTS repository listed above.

<br/><br/><br/><br/>
Whitted<br/>
![Alt text](Images/Whitted.png?raw=true "Whitted")
<br/><br/><br/><br/>

Stanford Bunny<br/>
![Alt text](Images/Bunny.png?raw=true "Stanford Bunny")

<br/><br/><br/><br/>
ThunkTS<br/>
![Alt text](Images/ThunkTS.png?raw=true "ThunkTS")
