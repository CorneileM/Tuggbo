# Tuggbo
## Type: Robot/Thingy
### An automated filament pulling device to control 3D-printer filament diameter when exiting a filament extruder

This repository contains circuit diagrams, 3D-printed parts, and Arduino sketches for a filament pulling device designed to control the diameter of 3D-printer filament
exiting an extruder. The diameter of the filament is controlled by the speed of a DC motor which pulls filament from an extruder (in this case, a Filabot EX2):
faster pulling = thinner filament; slower pulling = thicker filament. The speed of the DC motor is adjusted based on the measured filament diameter
which is read from a Mitutoyo digital plunge-dial indicator, to obtain a stable filament diameter. Because there's a lag period between filament exiting the extruder and being measured and an additional lag period between Tuggbo pulling speed adjustments and filament diameter change, I implemented a PID controller algorithm to obtain a stable filament diameter. 

The repository also includes an Arduino sketch and circuit diagram for a PID controlled heater that is placed inside the FILABOT EX2 hopper, to pre-heat and soften plastic pellets 
before entering the extrusion screw. This functions to prevent the the extrusion screw from being blocked by large pieces of hard plastic. This is specific to my application of producing filament from recycled plastic bottles. Preheating allowed for smoother plastic flow from the hopper into the extruder screw.

<img src="https://github.com/CorneileM/Tuggbo/blob/master/Images_and_video/Photo_of_Tuggbo.png" width="300"/>

### Useful resources and background information
#### PID process control
[Arduino PID library](https://playground.arduino.cc/Code/PIDLibrary/)

[PID explained for process engineers](https://www.aiche.org/resources/publications/cep/2016/february/pid-explained-process-engineers-part-2-tuning-coefficients)

