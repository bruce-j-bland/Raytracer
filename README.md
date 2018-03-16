# Raytracer
This is a work in progress software raytracer built for a class project.

The raytracer currently implements a Phong illumination model, as well as shadows. Reflection, refraction, and texture mapping will be implemented in the future.

The raytracing process is multithreaded, and is capable of edge-aware supersampling.

The SDL2 library is used for displaying the generated pixel maps.

Running the command line program allows you to select between one of three scenes: a work in progress reproduction of the first ray traced image by Whitted, a rendering of the Stanford Bunny, and a boardstate from ThunkTS (https://github.com/bruce-j-bland/ThunkTS).

The Standford Bunny implements a simple ply parser and uses a K-D Tree spacial data structure to handle the computationaly complex object.

The ThunkTS scene is procedurally generated from a boardstate, and makes use of a bespoke spatial data structure. The image is object-aware, and clicking on any grid in the scene will visibly select every pixel in that object.
<br/><br/><br/><br/>
Whitted<br/>
![Alt text](Images/Whitted.png?raw=true "Whitted")
<br/><br/><br/><br/>

Stanford Bunny<br/>
![Alt text](Images/Bunny.png?raw=true "Stanford Bunny")

<br/><br/><br/><br/>
ThunkTS<br/>
![Alt text](Images/Whitted.png?raw=true "ThunkTS")
