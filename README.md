# Temperature Gradient Microscopy Stage


**This repository is still in the making, please check back later.**

<a href="https://softliv.mat.ethz.ch/"><img src="https://softliv.mat.ethz.ch/_jcr_content/orgbox/image.imageformat.logo.1477473979.png" align = right height = 100px></a>
Microscopy stage with two temperature controlled metal block and a translational frame to move a sample in the temperature gradient by [Dominic Gerber](https://softliv.mat.ethz.ch/people/person-detail.MTk2MDQ5.TGlzdC8yMTgxLDIwMDIyNzA3NTg=.html) from the [Soft and Living Materials Group at ETH Zürich](https://softliv.mat.ethz.ch/).

Detailed instructions to build the setup can be found in the [wiki](https://github.com/gerberli/temperature_gradient_microscopy_stage/wiki/).

  > [Building your own setup](https://github.com/dogerber/temperature_gradient_microscopy_stage/wiki/Building-your-own-Setup) 

  > [User manual (softliv group)](https://github.com/gerberli/temperature_gradient_microscopy_stage/wiki/Operation-manual)





# Setup overview
The setup consists of a PID control loop, where an Arduini microcontroller regulated the voltage for two peltier devices. Thermistors are placed in two copper blocks of variable spacing above the peltier devices and the opposite side of the peltier devices is held at room temperature by water cooling. The sample is placed on the copper block and can be moved relative to the temperature gradient by a linear actuator.

<img src="images/picture_controller.png" height =300px > <img src="images/Exploded_view_animation.gif" height =300px >

# Features
* 0.05 K temperature accuracy, when calibrated
* 0.01 K temperature stability
* Temperature range -20 to 60 °C (when cooled with room temperature coolant)
* Max. cooling speed of 50 K/min 
* Variable spacing between metal blocks of 0-1 cm
* Translation of sample in the temperature gradient with 1.9 cm travel range and 300 nm stepsize
* Optional datalogging to microSD-card
* Optimized for microscope stages with 160x110 mm opening "Universal insert" [I-3091](https://www.asiimaging.com/products/stages-inserts/160-x-110-mm-slide-inserts/), but can be adapted

# Acknowledgement
Lawrence Wilen, Christian Furrer and Robert Style helped with the design of this stage and controller. The menu system used is [GEM](https://github.com/Spirik/GEM) by [Spirik](https://github.com/Spirik).

