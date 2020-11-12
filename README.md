# Tuggbo
### An automated filament pulling device to control 3D-printer filament diameter when exiting a filament extruder

This repository contains circuit diagrams, 3D-printed parts, and an Arduino sketch for a filament pulling device designed to control the diameter of 3D-printer filament
exiting an extruder. The diameter of the filament is controlled by the speed of a DC motor which pulls filament from an extruder (in this case, a Filabot EX2):
faster pulling = thinner filament; slower pulling = thicker filament. The speed of the DC motor is adjusted based on the measured filament diameter
which is read from a Mitutoyo digital plunge-dial indicator, to obtain a stable filament diameter.
