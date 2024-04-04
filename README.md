# Lab 06: Analog Signals and the ADC/DAC

Authors : Kyle G. Gayliyev <br>
Date: 25-March-2024<br>
Course: ECE 6780 - Embedded System Design, ECE Department, The University of Utah<br>
GitHub IDs: ggayliye <br>
Repo: https://github.com/ggayliye/Lab6_ECE6780Embedded_SR24 <br>
Date: By 28-March-2024, 11:59pm (Time of when submission is due/will be ready to be evaluated)<br>
Copyright: ECE 6780, Kyle G. Gayliyev - This work may not be copied for use in Academic Coursework.

## Overview of the Lab 06

The exercises in this section introduce the basic operation of the ADC and DAC peripherals. Each exercise is standalone; you may, however, implement them together in the main application loop without conflict.

### Required Materials:
* STM32F072 Discovery board
* oscilloscope
* potentiometer
* jumper wires

Lab 6 is consisted of 2 Parts:<br>

* Part 1: Measuring a Potentiometer With the ADC.
* Part 2: Generating Waveforms with the DAC.


### Part 1: Measuring a Potentiometer With the ADC.
The goal of this exercise is to use the ADC to measure the position of a potentiometer and display the result using the LEDs on the Discovery board. Each LED will have a threshold voltage/value that will cause it to turn on if the measured output of the ADC exceeds that value; likewise, they should turn off if the value drops below the threshold. When turning the potentiometer so that the output (center) pin’s voltage increases, the LEDs should light up in sequence.<br>

Instructions:<br>

1. Initialize the LED pins to output. 
2. Select a GPIO pin to use as the ADC input. 
* Remember that the “ADC_INx” additional/analog function indicates which ADC input channel the pin connects to. 
* Configure the pin to analog mode, no pull-up/down resistors. 
* Connect the output (center pin) of a potentiometer to the input pin. The other two pins of the potentiometer should be connected to 3V and GND. 
3. Enable the ADC1 in the RCC peripheral. 
4. Configure the ADC to 8-bit resolution, continuous conversion mode, hardware triggers disabled (software trigger only). 
5. Select/enable the input pin’s channel for ADC conversion. 
6. Perform a self-calibration, enable, and start the ADC. 
* The lab manual describes the basic procedure without mentioning the actual flags and conditions for advancement. 
* Use sections 13.4.1 (Calibration) and 13.4.2 (ADC on-off control) in the peripheral reference manual. 
7. In the main application loop, read the ADC data register and turn on/off LEDs depending on the value. 
* Use four increasing threshold values, each LED should have a minimum ADC value/- voltage to turn on. 
* As the voltage on the input pin increases, the LEDs should light one-by-one. 
* If the pin voltage decreases below the threshold for a LED, it should turn off.

### Part 2: Generating Waveforms with the DAC.
The goal of this exercise is to generate an analog waveform that can be viewed using either an oscilloscope or the analog input of a Saleae logic analyzer.
Instructions:<br>

1. Select a GPIO pin to use as the DAC output. 
* Remember that the “DAC_OUTx” additional/analog function indicates which DAC output channel the pin connects to. 
* Configure the pin to analog mode, no pull-up/down resistors. 
* Connect an oscilloscope probe or channel 0 of a Saleae logic analyzer to the pin.
* Remember that the old Saleae 16’s available for checkout do not have analog capabilities. Unless you have a new-model logic analyzer, you will need to use an oscilloscope.

2. Set the used DAC channel to software trigger mode. 
3. Enable the used DAC channel. 
4. Copy one of the wave-tables in figure 6.8 into your application. 
* The wave tables are 32-element arrays of unsigned 8-bit values. 
* Don’t use the square-wave. Pick one of the sine, triangle, or sawtooth waveforms. 
5. In the main application loop, use an index variable to write the next value in the wave-table (array) to the appropriate DAC data register. 
* Use the one that matches closest to the value type of the wave-table. 
6. Use a 1ms delay between updating the DAC to new values. 
* The resulting frequency of the waveform will be: 1 kHz (1ms between updates) / 32 samples per cycle / 31 Hz 
7. Submit a screen capture or photograph of the oscilloscope or logic capture in your postlab.


#### Exercise Specifications


<pre><ins>Future extensions</ins> :  There will be no future additions to this project. </pre>

# Partnership

We're partnered in the lab with students of two, but each student is required to complete
their work individually.

# Progress Notes

<ins>1st Week Notes:</ins> <br>

On Mon, 25th of March-2024, these tasks are completed:
- Created the "Lab6_ECE6780Embedded_SR24" GitHub repo.
- Created the "lab06" project using STM32CubeMX Software

<ins>2nd Week Notes:</ins> <br>

- It was a challenging lab.
## Testing
No Unit Test files are created as the nature of the project. Manual testing 
are performed in each step to ensure code improvements. Check "Testing Instructions"
section below.

# Time Expenditures:
<pre>Lab06: Predicted Hours: 12h		Actual Hours:	12h		 </pre>

The actual hours recorded on top reflect the time spent for the assignment including the time 
spent in labs. It excludes time spent reading and understanding the lab assignment instructions 
at the beginning of the lab (pre-lab work).

# Comments to Evaluators:




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I am uploading the full project files in this repo. 
Therfore, you may not need some of the information below to
prepaire this project to run. However, I'm still including those parts below.


<em>To able to fully test the main.c, other files and tools are required.<br>
For example, I created the project using the STM32CubeMX software first. Then <br>
clicked "Code Generation" button from the top menu after adjusting necessary <br>
settings. The instructions on how to adjust the settings will be given below. Then, <br>
the software automatically opens the Keil uVision5 softwhere where we code main.c.<br>
The main.c will be located under the "Application/User/Core" folder on the left menu of 
the Keil uVision5 softwhere.</em><br>

The main.c includes the 1st and 2nd part of the assignment. One of the assignments<br>
must be commented out. To test the commented out part, you'll need to uncomment that <br>
section and comment out the already uncommented section. Follow the comments<br>
out the sections in the main.c file.

## Testing Instructions:
After reading the discussion above, let's adjust the settings of
STM32CubeMX and Keil uVision5 softwares. <br><br>
STM32CubeMX:<br>

Select STM32F0 in the Series filter.<br>
* Select STM32F0x2 in the Lines filter
* Select LQFP64 in the Package filter <br>
At this point, there should only be a few choices available, select STM32F072RBTx and press the OK
button.<br>
Come to Project ->Menu->Settings.<br>
Name the new project. Select a directory where STMCube can create subfolders to store project files.<br>
Change the Toolchain/IDE dropdown menu to MDK-ARM V5.<br>
On the Project tab, move to the Code Generator tab at the top of the window.<br>
STMCube may take a while to copy the files to the directory specified in the settings. Afterward,
you may be asked if you want to open the project folder or project file itself. Click "project file".<br>
Now you should be in the "Keil uVision5" program as it's opened automatically.<br><br>

Keil uVision5:<br>

Click "Flash" -> "Configure flash tools" from the top menu.<br>
Click "Target" from the top menu. Find "Arm complier" menu and select 
"use default compiler version..".


One the setting is done, replace the main.c file in the "Application/User/Core" folder <br>
with my main.c file you downloaded.<br>
From the top menu, "Project"->"Build Target".<br>
Plug in your STM32F072 Discovery Microcontroller to your computer. <br>
Click "Flash" -> "Download" from the top menu. Test it on your STM32F072 Discovery Microcontroller.

Thank you for evaluating this project and providing feedback. <br>

Have a wonderful day!

# Consulted Peers:
N/A

# Caution/Warnings
* One common source of confusion when first using the ADC is that many of the bits in the control register are not only used to control the peripheral, but are also modified by hardware as status flags. 

* An example of this is the self-calibration. Once the user triggers a calibration cycle by setting the appropriate bit, the ADC will clear that same bit when calibration is complete. Most of the bits in the control register have strict requirements for when the ADC will allow you to set them. You will need to read the bit descriptions to know what you are allowed to configure before/after each step.



# Examples of Good Software Practice (GSP)
<pre><ins>DRY</ins> :</pre>
DRY is an abbreviation of "Don't repeat yourself". It's a principle of Software Engineering that
has the goal of reducing repetition and redundancy in the coding. Using abstractions and normalization
are advised over redundancy <a href="https://en.wikipedia.org/wiki/Don%27t_repeat_yourself">[2]</a>.

<pre><ins>Separation of Concerns</ins> :</pre>
This concept is similar to the "divide and conquer" principle where you divide
a big software project into small projects to complete. A software design technique that focuses on separating 
and freeing functionalities of a program is called Modular programming. Making each of the divided small 
projects independent and addressing a separate concern, it'll make it easy to reduce, detect 
and fix the errors. <a href="https://en.wikipedia.org/wiki/Separation_of_concerns">[3]</a>

<pre><ins>Good Code Typing Practices</ins> :</pre>
Good coding practices can be listed as: Naming variables with a relevant name, commenting 
in between code lines with a brief explanation if the code is not self-explanatory, creating 
helper methods that can be used multiple times and by other sections.


<pre><ins>Testing Strategies</ins> :</pre>
Unit Testing can be summarized in 3 types of techniques:<br>

1. <ins>Black Box Testing : </ins> In this testing, input, user interface, and output parts are covered.
2. <ins>White Box Testing : </ins> In this testing, functionality, design structure, and code models are covered.
3. <ins>Gray Box Testing : </ins> In this testing, analysis of code performance, relevant test cases,
methods, and functions are covered.<a href="https://www.geeksforgeeks.org/unit-testing-software-testing/">[4]</a>


# References:
1. Canvas Page Class Materials.(For example, lecture slides, additional resources and pdf files). <br>
2. https://en.wikipedia.org/wiki/Don%27t_repeat_yourself<br>
3. https://en.wikipedia.org/wiki/Separation_of_concerns<br>
4. https://www.geeksforgeeks.org/unit-testing-software-testing/<br>






