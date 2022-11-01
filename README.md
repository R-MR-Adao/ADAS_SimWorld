# ADAS SimWorld

![ADAS_SimWorld_wide](https://user-images.githubusercontent.com/111191306/198900690-c87a708c-2ea7-41a6-b735-091224d945b8.png)


SimWorld is a synthetic data egenration tool for the simulation of sensor-based object detection and tracking araound an ego vechicle in an [Advanced Driver Assistance System (ADAS)](https://en.wikipedia.org/wiki/Advanced_driver-assistance_system).
It was designed for mainly for didatic purposes,and the prototyping of simple detection and tracking algortihms.

<p align="center">
<img src="doc/videos/ADAS_SimWorld_basic.gif" />
</p>

## Table Contents
 - [Introduction](@introduction)
 - [Graphical User Interface](@graphicaluserinterface)
   - [Main control panel](@maincontrolpanel)
   - [Simulation visualizations](@simulationvisualizations)
   - [User code editor](@rsercodeeditor)
   - [Speedometer widget](@speedometerwidget)
- [Data generation model](@datagenerationmodel)
- [SimWorld inhabitants](@simworldinhabitants)
- [User solutions](@usersolutions)
- [Widgets](@widgets)
- [A peek under the hood](@apeekunderthehood)
  - [Software architecture](@softwarearchitecture)

## Introduction

ADAS SimWorld is a simple MATLAB toolbox for the quick visualization of synthethically-generated sensor detections and the prototyping of simple objetc tracking and classification algorithms.
It simulates the motion of an ego vehicle along a (periodic) road,using four arbitrary sensors to analyze its surroundings.

All data is generated "on the fly", using a set of base equations.
As the ego vehicle progresses through the road, it comes accross both stationart and moving objects, which get detected by the surrounding sensors.

This tool provides a simple Graphical User Interface (GUI), which offsers a visualization of the real-time simulated world, the object detections in each of the surrounding four sensors' field of view, and a text editor to implement the tracking algorithm code.
A detailed description of the GUI can be found in the [Graphical User Interface](@graphicaluserinterface) section.

![ADAS_SimWorld_RoadMapTopView](https://user-images.githubusercontent.com/111191306/199146067-70effa9f-62b3-4fd1-beb6-2a39eb8d4650.png)

## Graphical User Interface

The GUI is divided into 5 components, as illustrated in the figure below:
 1. [Main control panel](@maincontrolpanel)
 2. [Simulation visualizations](@simulationvisualizations)
 3. Sensors' Field of View (FoV)
 4. [User code editor](@rsercodeeditor)
 5. [Speedometer widget](@speedometerwidget)

![ADAS_SimWorld_GUI](https://user-images.githubusercontent.com/111191306/199229682-0ac34436-9d05-49e0-834b-fadfa18b4472.png)

### Main control panel

The main control panel allows reseting, playing and stepping the simulation, as well as setting some basic configurations.
These include toggling over the active sensors, and setting a few render parameters (such as speed and resolution).

The sensor acivation/deactivation is illustrated in the video below.

<p align="center">
<img src="doc/videos/ADAS_SimWorld_GUI_chooseSensors.gif" />
</p>

### Simulation visualizations

These allow setting the 3D perpective view over the simulation space.
The video below illustrates how the four sliders surrounding the 3rd person simulation visualizaition panel can be used to control the zoom (top slider), tilt (right slider), pan (left slider) and rotation (bottom slider).

<p align="center">
<img src="doc/videos/ADAS_SimWorld_GUI_views.gif" />
</p>

### User code editor

The GUI offers a simple text editor for the user to implement and quickly test their code.
As of version `1.0`, the user code API includes only the function `reconstruct_360_space`, which is meant to reconstruct the sensor readings from their sensor-specific coordinate systems to the global ego reference.
Two solutions are provided for this code, with and without object classification algorithmalgorithms.
The GUI is prepared to host more than one code file to implement different functionalities. The code editor allows choosing the file by a drop down menu at the top-left of the _Main playback_ panel.

More information on the implementations can be found in [User solutions](@usersolutions) section.

<p align="center">
<img src="doc/videos/ADAS_SimWorld_userCode.gif" />
</p>

### Speedometer widget

Widget modules can be added to the code as extended visualization tool.
A simple speedometer widget is included as part of the base software as an example of an external add-on.
In this case, this speedometer displayes the instant ego speed in km/h, as well as the cumulative travelled distance.

<p align="center">
<img src="doc/videos/ADAS_SimWorld_widgetSpeedometer.gif" />
</p>

## Data generation model

All SimWorld data is synthetically generated in real-time as the simulation runs.
This section highlights the main principles and equations for the data generation.

### Road model

The ego's progression along the SimWorld road is obtained from a very simple set of principles and equations.

#### Base principles:
 - The road shape is defined by via a periodic parametrical equation.
 - Every object on the road and the elements that build the road itself (e.g. lanes) are governed by the road equation.
 - Each moving object (including the ego vehicle) moved along the road by a fix time dpendence equation.
 - The road's parameterization vector is the only pre-established data array. The position and dynamic properties of every moving objects is then deterministically calculated for each moment in time, from the setp of equations that define their time-dependent p+roperties.
 - The 3rd-person view of SimWorld in the perspective of the ego vehicle is obtained by roatiting every environment element around the ego, by and angle $\theta$ defined by orientation of the road at the ego vehicle's position.

#### Base equations:

The road equation is the main SimWorld defining function.
In its most general form, the road shape $F_{\mathrm{road}}$ can be defined as:

$$
F_{\mathrm{road}}(s) =
\begin{bmatrix}
f_\mathrm{road}^{(x)}(s)\\
f_\mathrm{road}^{(y)}(s)\\
f_\mathrm{road}^{(z)}(s)
\end{bmatrix}
$$

where $s$ is a geometrical parameterization, and $f_\mathrm{road}^{(j)}(s),\ j = \{x,y,z\}$ are the road's cartesian component functions.
Programmatically, $s$ is defined by an array, while $F_{\mathrm{road}}$ is defined by a set of equations.

For simplicity, this base implementation defines $f_\mathrm{road}^{(z)}(s)$ as null, and $f_\mathrm{road}^{(y)}(s)$ as a function of its $f_\mathrm{road}^{(x)}(s)$, i.e., 

$$
F_{\mathrm{road}}(s) =
\begin{bmatrix}
s\\
f_\mathrm{road}^{(y)}(s)\\
0
\end{bmatrix}
$$

or in other words, $f_\mathrm{road}^{(x)}(s) = x$ and $f_\mathrm{road}^{(y)}(s)=f_\mathrm{road}^{(y)}(x)$.

As mentioned in ther [Base principles](@baseprinciples), $f_\mathrm{road}^{(y)}(x)$ must be periodic.
Moreover, the range of the paramterization array $s$ must be an integer multiple of the periodicity of $F_{\mathrm{road}}$.
This base implementation uses the following function, which complies with these requirements:

$$
f_\mathrm{road}^{(y)}(x) = \frac{T}{3}\sin \left(\frac{2\pi}{T}x\right) \cos \left(\frac{2\pi}{T/3}x\right)
$$

where $T$ is the road period, and thus $s\in\left[nT ,mT \right],\ n < m\ \mathrm{and}\ n,m\in\mathbb{Z}$.

It isimportant to notice that $f_\mathrm{road}^{(y)}(x)$ only defines the _center_ of the road.
Since moving objects do not all move along the center, each road lane must be defined along a perpendicular expansion of $f_\mathrm{road}^{(y)}(x)$.
This can be achieved by calculating the vector $U(s)$ that is perpendicular to $F_{\mathrm{road}}(s)$ along the $s$ parameterization,

$$
U(s) = 
\begin{bmatrix}
u_x(s)\\
u_y(s)
\end{bmatrix} =
\frac{\mathrm{d}}{\mathrm{d}s} 
\begin{bmatrix}
-f_\mathrm{road}^{(y)}(s)\\
f_\mathrm{road}^{(x)}(s)
\end{bmatrix}
$$

which, in its normalized form $U_n$,

$$
U_n(s) = 
\frac{U(s)}{\sqrt{u_x^2(s) + u_y^2(s)}}
$$

can be used to express the function $F_\mathrm{lane}$ for a lane displace from the road center by the distance $w$ as:

$$
F_\mathrm{lane}(s) =
\begin{bmatrix}
f_\mathrm{lane}^{(x)}(s)\\
f_\mathrm{lane}^{(y)}(s)
\end{bmatrix} =
F_\mathrm{road}(s) + wU_n(s)
$$

so that, effectively,

$$
F_\mathrm{lane}(s) = 
\begin{bmatrix}
f_\mathrm{road}^{(x)} - w \frac{\mathrm{d}}{\mathrm{d}s} f_\mathrm{road}^{(y)} \Bigg/ \sqrt{ \left(\frac{\mathrm{d}}{\mathrm{d}s} f_\mathrm{road}^{(x)}\right)^2 + \left(\frac{\mathrm{d}}{\mathrm{d}s} f_\mathrm{road}^{(y)}\right)^2 }\\
f_\mathrm{road}^{(y)} + w \frac{\mathrm{d}}{\mathrm{d}s} f_\mathrm{road}^{(x)} \Bigg/ \sqrt{ \left(\frac{\mathrm{d}}{\mathrm{d}s} f_\mathrm{road}^{(x)}\right)^2 + \left(\frac{\mathrm{d}}{\mathrm{d}s} f_\mathrm{road}^{(y)}\right)^2 }
\end{bmatrix}
$$

where the $s$ dependencies on the right side of the equals sign are omitted for sintax simplicity.
The figure below illustrates the lane widening obtained using the equations above.

<p align="center">
<img height=250 src="https://user-images.githubusercontent.com/111191306/199353113-cd71cc65-7dce-4f77-adb2-0413c04c5f46.png">
</p>

As shown ahead in the [Time progression and 3rd person perspective](@timeprogressionand3rdpersonperspective) section, $F_\mathrm{lane}(s)$ is used not only to draw the road edges, but also to define the trajectory of each moving object.

### Time progression and 3rd person perspective

### 3D Terrain model

## SimWorld inhabitants

### Stationary objects

### Moving objects

## User solutions

## Widgets

## A peek under the hood

### Software architecture
