Final Project using SUMO



Requirements :

SUMO - https://eclipse.dev/sumo/

Python

TraCI library - https://sumo.dlr.de/docs/TraCI.html


11/3/2025 -
Created a generic 5-lane highway that can be used to start testing. Simulation runs with cars going across. sim.py is initially using example code from (https://www.youtube.com/watch?v=bbAWNb1j1w0) and (https://github.com/RoadwayVR/SUMO-Traffic-Simulator-Tutorial/blob/main/Traci1.py). It is not implemented yet, but is the next part to work on.

The main workflow will be the following:

  .sumocfg file and the xml has all the settings for the simulation. Traci is a python library that allows for interaction with the simuation in each "step". We can have the script go though each car that enters the highway and create a "car" object. This object can have lots of different variables ( postion, speed, intention, etc.). From here we can have cars want to change lanes at different times and create the "algorithm" that handles negotiations between nearby cars. From there we can create the hacked version and measure the result and then protect against it.

  For right now you can run the simulation without the python script using "sumo-gui -c highway.sumocfg
